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

/* Use local variable stub for DP/AP registers. */
static uint32_t dp_ctrl_stat;
static uint32_t dp_id_code;
static uint32_t ap_sel, ap_bank;

/* Dmem file handler. */
static int dmem_fd = -1;

/* DAP error code. */
static int dmem_dap_retval = ERROR_OK;

/* Default dmem device. */
#define DMEM_DEV_PATH_DEFAULT	"/dev/mem"
static char *dmem_dev_path;
static uint64_t dmem_dap_base_address;
static uint8_t dmem_dap_max_aps = 1;
static uint32_t dmem_dap_ap_offset = 0x100;

static int dmem_dp_q_read(struct adiv5_dap *dap, unsigned int reg,
			   uint32_t *data)
{
	LOG_ERROR("I am here: %s\n", __func__);
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
	LOG_ERROR("I am here: %s\n", __func__);
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
	LOG_ERROR("I am here: %s\n", __func__);
	int rc = ERROR_OK;

	if (is_adiv6(ap->dap)) {
		static bool error_flagged;
		if (!error_flagged)
			LOG_ERROR("ADIv6 dap not supported by dmem dap-direct mode");
		error_flagged = true;
		return ERROR_FAIL;
	}

	switch (reg) {
	case ADIV5_MEM_AP_REG_CSW:
	case ADIV5_MEM_AP_REG_CFG:
	case ADIV5_MEM_AP_REG_BASE:
	case ADIV5_AP_REG_IDR:
	case ADIV5_MEM_AP_REG_BD0:
	case ADIV5_MEM_AP_REG_BD1:
	case ADIV5_MEM_AP_REG_BD2:
	case ADIV5_MEM_AP_REG_BD3:
	case ADIV5_MEM_AP_REG_DRW:
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
	LOG_ERROR("I am here: %s\n", __func__);
	int rc = ERROR_OK;

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
	case ADIV5_MEM_AP_REG_TAR:
	case ADIV5_MEM_AP_REG_BD0:
	case ADIV5_MEM_AP_REG_BD1:
	case ADIV5_MEM_AP_REG_BD2:
	case ADIV5_MEM_AP_REG_BD3:
	case ADIV5_MEM_AP_REG_DRW:
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
	LOG_ERROR("I am here: %s\n", __func__);
	return ERROR_OK;
}

static int dmem_dp_run(struct adiv5_dap *dap)
{
	LOG_ERROR("I am here: %s\n", __func__);
	int retval = dmem_dap_retval;

	/* Clear the error code. */
	dmem_dap_retval = ERROR_OK;

	return retval;
}

static int dmem_connect(struct adiv5_dap *dap)
{
	LOG_ERROR("I am here: %s\n", __func__);
	char *path = dmem_dev_path ? dmem_dev_path : DMEM_DEV_PATH_DEFAULT;

	dmem_fd = open(path, O_RDWR | O_SYNC);
	if (dmem_fd == -1) {
		LOG_ERROR("Unable to open %s\n", path);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void dmem_disconnect(struct adiv5_dap *dap)
{
	LOG_ERROR("I am here: %s\n", __func__);
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
