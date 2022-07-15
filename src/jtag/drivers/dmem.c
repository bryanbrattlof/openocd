/*
 * Copyright (c) 2020, Mellanox Technologies Ltd. - All Rights Reserved
 * Liming Sun <lsun@mellanox.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/types.h>
#include <helper/system.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <jtag/interface.h>
#ifdef HAVE_SYS_IOCTL_H
#include <sys/ioctl.h>
#endif
#include <target/arm_adi_v5.h>
#include <transport/transport.h>

/* Rshim channel where the CoreSight register resides. */
#define RSH_MMIO_CHANNEL_RSHIM	0x1

/* APB and tile address translation. */
#define RSH_CS_ROM_BASE		0x80000000
#define RSH_CS_TILE_BASE	0x44000000
#define RSH_CS_TILE_SIZE	0x04000000

/*
 * APB-AP Identification Register
 * The default value is defined in "CoreSight on-chip trace and debug
 * (Revision: r1p0)", Section 3.16.5 APB-AP register summary.
 */
#define APB_AP_IDR			0x44770002

/* CoreSight register definition. */
#define RSH_CORESIGHT_CTL		0x0e00
#define RSH_CORESIGHT_CTL_GO_SHIFT	0
#define RSH_CORESIGHT_CTL_GO_MASK	0x1ULL
#define RSH_CORESIGHT_CTL_ACTION_SHIFT	1
#define RSH_CORESIGHT_CTL_ACTION_MASK	0x2ULL
#define RSH_CORESIGHT_CTL_ADDR_SHIFT	2
#define RSH_CORESIGHT_CTL_ADDR_MASK	0x7ffffffcULL
#define RSH_CORESIGHT_CTL_ERR_SHIFT	31
#define RSH_CORESIGHT_CTL_ERR_MASK	0x80000000ULL
#define RSH_CORESIGHT_CTL_DATA_SHIFT	32
#define RSH_CORESIGHT_CTL_DATA_MASK	0xffffffff00000000ULL

/* Util macros to access the CoreSight register. */
#define RSH_CS_GET_FIELD(reg, field) \
	(((uint64_t)(reg) & RSH_CORESIGHT_CTL_##field##_MASK) >> \
		RSH_CORESIGHT_CTL_##field##_SHIFT)

#define RSH_CS_SET_FIELD(reg, field, value) \
	(reg) = (((reg) & ~RSH_CORESIGHT_CTL_##field##_MASK) | \
		(((uint64_t)(value) << RSH_CORESIGHT_CTL_##field##_SHIFT) & \
		RSH_CORESIGHT_CTL_##field##_MASK))

#ifdef HAVE_SYS_IOCTL_H
/* Message used to program dmem via ioctl(). */
typedef struct {
	uint32_t addr;
	uint64_t data;
} __attribute__((packed)) dmem_ioctl_msg;

enum {
	RSH_IOC_READ = _IOWR('R', 0, dmem_ioctl_msg),
	RSH_IOC_WRITE = _IOWR('R', 1, dmem_ioctl_msg),
};
#endif

/* Use local variable stub for DP/AP registers. */
static uint32_t dp_ctrl_stat;
static uint32_t dp_id_code;
static uint32_t ap_sel, ap_bank;
static uint32_t ap_csw;
static uint32_t ap_drw;
static uint32_t ap_tar, ap_tar_inc;

/* Static functions to read/write via dmem/coresight. */
static int (*dmem_read)(int chan, int addr, uint64_t *value);
static int (*dmem_write)(int chan, int addr, uint64_t value);
static int coresight_write(uint32_t tile, uint32_t addr, uint32_t wdata);
static int coresight_read(uint32_t tile, uint32_t addr, uint32_t *value);

/* RShim file handler. */
static int dmem_fd = -1;

/* DAP error code. */
static int dmem_dap_retval = ERROR_OK;

/* Default dmem device. */
#define DMEM_DEV_PATH_DEFAULT	"/dev/mem"
static char *dmem_dev_path;
static uint64_t dmem_dap_base_address;
static uint8_t dmem_dap_max_aps = 1;
static uint32_t dmem_dap_ap_offset = 0x100;

static int dmem_dev_read(int chan, int addr, uint64_t *value)
{
	int rc;

	addr = (addr & 0xFFFF) | (1 << 16);
	rc = pread(dmem_fd, value, sizeof(*value), addr);

#ifdef HAVE_SYS_IOCTL_H
	if (rc < 0 && errno == ENOSYS) {
		dmem_ioctl_msg msg;

		msg.addr = addr;
		msg.data = 0;
		rc = ioctl(dmem_fd, RSH_IOC_READ, &msg);
		if (!rc)
			*value = msg.data;
	}
#endif

	return rc;
}

static int dmem_dev_write(int chan, int addr, uint64_t value)
{
	int rc;

	addr = (addr & 0xFFFF) | (1 << 16);
	rc = pwrite(dmem_fd, &value, sizeof(value), addr);

#ifdef HAVE_SYS_IOCTL_H
	if (rc < 0 && errno == ENOSYS) {
		dmem_ioctl_msg msg;

		msg.addr = addr;
		msg.data = value;
		rc = ioctl(dmem_fd, RSH_IOC_WRITE, &msg);
	}
#endif

	return rc;
}

/* Convert AP address to tile local address. */
static void ap_addr_2_tile(int *tile, uint32_t *addr)
{
	*addr -= RSH_CS_ROM_BASE;

	if (*addr < RSH_CS_TILE_BASE) {
		*tile = 0;
	} else {
		*addr -= RSH_CS_TILE_BASE;
		*tile = *addr / RSH_CS_TILE_SIZE + 1;
		*addr = *addr % RSH_CS_TILE_SIZE;
	}
}

/*
 * Write 4 bytes on the APB bus.
 * tile = 0: access the root CS_ROM table
 *      > 0: access the ROM table of cluster (tile - 1)
 */
static int coresight_write(uint32_t tile, uint32_t addr, uint32_t wdata)
{
	uint64_t ctl = 0;
	int rc;

	if (!dmem_read || !dmem_write)
		return ERROR_FAIL;

	/*
	 * ADDR[28]    - must be set to 1 due to coresight ip.
	 * ADDR[27:24] - linear tile id
	 */
	addr = (addr >> 2) | (tile << 24);
	if (tile)
		addr |= (1 << 28);
	RSH_CS_SET_FIELD(ctl, ADDR, addr);
	RSH_CS_SET_FIELD(ctl, ACTION, 0);	/* write */
	RSH_CS_SET_FIELD(ctl, DATA, wdata);
	RSH_CS_SET_FIELD(ctl, GO, 1);		/* start */

	dmem_write(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = dmem_read(RSH_MMIO_CHANNEL_RSHIM,
				RSH_CORESIGHT_CTL, &ctl);
		if (rc < 0) {
			LOG_ERROR("Failed to read dmem.\n");
			return rc;
		}
	} while (RSH_CS_GET_FIELD(ctl, GO));

	return ERROR_OK;
}

static int coresight_read(uint32_t tile, uint32_t addr, uint32_t *value)
{
	uint64_t ctl = 0;
	int rc;

	if (!dmem_read || !dmem_write)
		return ERROR_FAIL;

	/*
	 * ADDR[28]    - must be set to 1 due to coresight ip.
	 * ADDR[27:24] - linear tile id
	 */
	addr = (addr >> 2) | (tile << 24);
	if (tile)
		addr |= (1 << 28);
	RSH_CS_SET_FIELD(ctl, ADDR, addr);
	RSH_CS_SET_FIELD(ctl, ACTION, 1);	/* read */
	RSH_CS_SET_FIELD(ctl, GO, 1);		/* start */

	dmem_write(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = dmem_read(RSH_MMIO_CHANNEL_RSHIM,
				RSH_CORESIGHT_CTL, &ctl);
		if (rc < 0) {
			LOG_ERROR("Failed to write dmem.\n");
			return rc;
		}
	} while (RSH_CS_GET_FIELD(ctl, GO));

	*value = RSH_CS_GET_FIELD(ctl, DATA);
	return ERROR_OK;
}

static int dmem_dp_q_read(struct adiv5_dap *dap, unsigned int reg,
			   uint32_t *data)
{
	if (!data)
		return ERROR_OK;

	switch (reg) {
	case DP_DPIDR:
		*data = dp_id_code;
		break;

	case DP_CTRL_STAT:
		*data = CDBGPWRUPACK | CSYSPWRUPACK;
		break;

	default:
		break;
	}

	return ERROR_OK;
}

static int dmem_dp_q_write(struct adiv5_dap *dap, unsigned int reg,
			    uint32_t data)
{
	switch (reg) {
	case DP_CTRL_STAT:
		dp_ctrl_stat = data;
		break;
	case DP_SELECT:
		ap_sel = (data & DP_SELECT_APSEL) >> 24;
		ap_bank = (data & DP_SELECT_APBANK) >> 4;
		break;
	default:
		LOG_INFO("Unknown command");
		break;
	}

	return ERROR_OK;
}

static int dmem_ap_q_read(struct adiv5_ap *ap, unsigned int reg,
			   uint32_t *data)
{
	uint32_t addr;
	int rc = ERROR_OK, tile;

	if (is_adiv6(ap->dap)) {
		static bool error_flagged;
		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");
		error_flagged = true;
		return ERROR_FAIL;
	}

	switch (reg) {
	case ADIV5_MEM_AP_REG_CSW:
		*data = ap_csw;
		break;

	case ADIV5_MEM_AP_REG_CFG:
		*data = 0;
		break;

	case ADIV5_MEM_AP_REG_BASE:
		*data = RSH_CS_ROM_BASE;
		break;

	case ADIV5_AP_REG_IDR:
		if (ap->ap_num == 0)
			*data = APB_AP_IDR;
		else
			*data = 0;
		break;

	case ADIV5_MEM_AP_REG_BD0:
	case ADIV5_MEM_AP_REG_BD1:
	case ADIV5_MEM_AP_REG_BD2:
	case ADIV5_MEM_AP_REG_BD3:
		addr = (ap_tar & ~0xf) + (reg & 0x0C);
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_read(tile, addr, data);
		break;

	case ADIV5_MEM_AP_REG_DRW:
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_read(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		break;

	default:
		LOG_INFO("Unknown command");
		rc = ERROR_FAIL;
		break;
	}

	/* Track the last error code. */
	if (rc != ERROR_OK)
		dmem_dap_retval = rc;

	return rc;
}

static int dmem_ap_q_write(struct adiv5_ap *ap, unsigned int reg,
			    uint32_t data)
{
	int rc = ERROR_OK, tile;
	uint32_t addr;

	if (is_adiv6(ap->dap)) {
		static bool error_flagged;
		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");
		error_flagged = true;
		return ERROR_FAIL;
	}

	if (ap_bank != 0) {
		dmem_dap_retval = ERROR_FAIL;
		return ERROR_FAIL;
	}

	switch (reg) {
	case ADIV5_MEM_AP_REG_CSW:
		ap_csw = data;
		break;

	case ADIV5_MEM_AP_REG_TAR:
		ap_tar = data;
		ap_tar_inc = 0;
		break;

	case ADIV5_MEM_AP_REG_BD0:
	case ADIV5_MEM_AP_REG_BD1:
	case ADIV5_MEM_AP_REG_BD2:
	case ADIV5_MEM_AP_REG_BD3:
		addr = (ap_tar & ~0xf) + (reg & 0x0C);
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_write(tile, addr, data);
		break;

	case ADIV5_MEM_AP_REG_DRW:
		ap_drw = data;
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_write(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		break;

	default:
		rc = EINVAL;
		break;
	}

	/* Track the last error code. */
	if (rc != ERROR_OK)
		dmem_dap_retval = rc;

	return rc;
}

static int dmem_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int dmem_dp_run(struct adiv5_dap *dap)
{
	int retval = dmem_dap_retval;

	/* Clear the error code. */
	dmem_dap_retval = ERROR_OK;

	return retval;
}

static int dmem_connect(struct adiv5_dap *dap)
{
	char *path = dmem_dev_path ? dmem_dev_path : DMEM_DEV_PATH_DEFAULT;

	dmem_fd = open(path, O_RDWR | O_SYNC);
	if (dmem_fd == -1) {
		LOG_ERROR("Unable to open %s\n", path);
		return ERROR_FAIL;
	}

	/*
	 * Set read/write operation via the device file. Function pointers
	 * are used here so more ways like remote accessing via socket could
	 * be added later.
	 */
	dmem_read = dmem_dev_read;
	dmem_write = dmem_dev_write;

	return ERROR_OK;
}

static void dmem_disconnect(struct adiv5_dap *dap)
{
	if (dmem_fd != -1) {
		close(dmem_fd);
		dmem_fd = -1;
	}
}

COMMAND_HANDLER(dmem_dap_device_command)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	free(dmem_dev_path);
	dmem_dev_path = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_base_address_command)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], dmem_dap_base_address);
	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_max_aps_command)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], dmem_dap_max_aps);
	return ERROR_OK;
}

COMMAND_HANDLER(dmem_dap_ap_offset_command)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], dmem_dap_ap_offset);
	return ERROR_OK;
}
COMMAND_HANDLER(dmem_dap_config_info_command)
{
	if (CMD_ARGC != 0) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	command_print(CMD,"dmem (Direct Memory) AP Adapter Configuration:");
	command_print(CMD," Device       : %s", dmem_dev_path ? dmem_dev_path : DMEM_DEV_PATH_DEFAULT);
	command_print(CMD," Base Address : 0x%lx", dmem_dap_base_address);
	command_print(CMD," Max APs      : %d", dmem_dap_max_aps);
	command_print(CMD," AP offset    : 0x%08x", dmem_dap_ap_offset);
	return ERROR_OK;
}

static const struct command_registration dmem_dap_subcommand_handlers[] = {
	{
		.name = "device",
		.handler = dmem_dap_device_command,
		.mode = COMMAND_CONFIG,
		.help = "set the dmem memory access device",
		.usage = "</dev/mem>",
	},
	{
		.name = "base_address",
		.handler = dmem_dap_base_address_command,
		.mode = COMMAND_CONFIG,
		.help = "set the dmem dap AP memory map base address",
		.usage = "<0x700000000>",
	},
	{
		.name = "max_aps",
		.handler = dmem_dap_max_aps_command,
		.mode = COMMAND_CONFIG,
		.help = "set the maximum number of APs this will support",
		.usage = "<1>",
	},
	{
		.name = "ap_address_offset",
		.handler = dmem_dap_ap_offset_command,
		.mode = COMMAND_CONFIG,
		.help = "set the offsets of each ap index",
		.usage = "<0x100>",
	},
	{
		.name = "info",
		.handler = dmem_dap_config_info_command,
		.mode = COMMAND_ANY,
		.help = "print the config info",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dmem_dap_command_handlers[] = {
	{
		.name = "dmem",
		.mode = COMMAND_ANY,
		.help = "Perform dmem (Direct Memory) DAP management and configuration",
		.chain = dmem_dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int dmem_dap_init(void)
{
	return ERROR_OK;
}

static int dmem_dap_quit(void)
{
	return ERROR_OK;
}

static int dmem_dap_reset(int req_trst, int req_srst)
{
	return ERROR_OK;
}

static int dmem_dap_speed(int speed)
{
	return ERROR_OK;
}

static int dmem_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int dmem_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

/* DAP operations. */
static const struct dap_ops dmem_dap_ops = {
	.connect = dmem_connect,
	.queue_dp_read = dmem_dp_q_read,
	.queue_dp_write = dmem_dp_q_write,
	.queue_ap_read = dmem_ap_q_read,
	.queue_ap_write = dmem_ap_q_write,
	.queue_ap_abort = dmem_ap_q_abort,
	.run = dmem_dp_run,
	.quit = dmem_disconnect,
};

static const char *const dmem_dap_transport[] = { "dapdirect_swd", NULL };

struct adapter_driver dmem_dap_adapter_driver = {
	.name = "dmem",
	.transports = dmem_dap_transport,
	.commands = dmem_dap_command_handlers,

	.init = dmem_dap_init,
	.quit = dmem_dap_quit,
	.reset = dmem_dap_reset,
	.speed = dmem_dap_speed,
	.khz = dmem_dap_khz,
	.speed_div = dmem_dap_speed_div,

	.dap_swd_ops = &dmem_dap_ops,
};
