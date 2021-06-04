#!/usr/bin/env python3.6
#
# Copyright (c) 2018 Texas Instruments Incorporated
#
# All rights reserved not granted herein.
#
# Limited License.
#
# Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
# license under copyrights and patents it now or hereafter owns or controls to make,
# have made, use, import, offer to sell and sell ("Utilize") this software subject to the
# terms herein.  With respect to the foregoing patent license, such license is granted
# solely to the extent that any such patent is necessary to Utilize the software alone.
# The patent license shall not apply to any combinations which include this software,
# other than combinations with devices manufactured by or for TI ("TI Devices").
# No hardware patent is licensed hereunder.
#
# Redistributions must preserve existing copyright notices and reproduce this license
# (including the above copyright notice and the disclaimer and (if applicable) source
# code license limitations below) in the documentation and/or other materials provided
# with the distribution
#
# Redistribution and use in binary form, without modification, are permitted provided
# that the following conditions are met:
#
#       No reverse engineering, decompilation, or disassembly of this software is
# permitted with respect to any software provided in binary form.
#
#       any redistribution and use are licensed by TI for use only with TI Devices.
#
#       Nothing shall obligate TI to provide you with source code for the software
# licensed and provided to you in object code.
#
# If software source code is provided to you, modification and redistribution of the
# source code are permitted provided that the following conditions are met:
#
#       any redistribution and use of the source code, including any resulting derivative
# works, are licensed by TI for use only with TI Devices.
#
#       any redistribution and use of any object code compiled from the source code
# and any resulting derivative works, are licensed by TI for use only with TI Devices.
#
# Neither the name of Texas Instruments Incorporated nor the names of its suppliers
#
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# DISCLAIMER.
#
# THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#
#

#
# This script is used to generate 'MEMORY' section for multiple CPUs
# in different CPU specific linker command files.
#
# This helps to define the memory map in one file vs having to manually
# keep the system memory map consistant across multiple CPUs
# in different linker commnd files.
#
# Make sure PyTI_PSDK_RTOS module is installed before running this script.
# See vision_apps/tools/PyTI_PSDK_RTOS/README.txt to install PyTI_PSDK_RTOS module.
#
# Edit this file to change the memory map
#
# Run this script by doing below,
# python3.5 gen_linker_mem_map.py
#
# This will generate linker command file at below folders
# ./<cpu name>/linker_mem_map.cmd
#
# Here CPU name is mpu1, c66x_1, c66x_2, c7x_1, mcu1_0, mcu2_0, mcu3_0
#
#
from ti_psdk_rtos_tools import *

KB = 1024;
MB = KB*KB;
GB = KB*MB;

#
# Notes,
# - recommend to keep all memory segment sizes in units of KB at least
#

#
# DDR, MSMC base and size
#
ddr_mem_addr  = 0x80000000;
ddr_mem_size  = 16*MB;
msmc_mem_addr = 0x70000000;

#
# MSMC memory allocation for various CPUs
#
mpu1_msmc_addr   = msmc_mem_addr;
mpu1_msmc_size   = 4*KB;


#
# DDR memory allocation for various CPUs
#
mpu1_ddr_addr   = ddr_mem_addr;
mpu1_ddr_size   = 16*MB;

#
# DDR memory allocation for various shared memories
#
app_log_mem_addr        = mpu1_ddr_addr         + mpu1_ddr_size;
app_log_mem_size        = 256*KB;

#
# Create memory section based on addr and size defined above, including
# any CPU specific internal memories
#


# MSMC memory sections
mpu1_msmc   = MemSection("MSMC_MPU1", "RWIX", mpu1_msmc_addr  , mpu1_msmc_size  , "MSMC for MPU1");


# CPU code/data memory sections in DDR
mpu1_ddr    = MemSection("DDR_MPU1", "RWIX", mpu1_ddr_addr  , mpu1_ddr_size  , "DDR for MPU1   for code/data");

# Shared memory memory sections in DDR
app_log_mem        = MemSection("APP_LOG_MEM"       , "", app_log_mem_addr       , app_log_mem_size       , "Memory for remote core logging");

#
# Create CPU specific memory maps using memory sections created above
#

mpu1_mmap = MemoryMap("mpu1");
mpu1_mmap.addMemSection( mpu1_msmc          );
mpu1_mmap.addMemSection( mpu1_ddr           );
mpu1_mmap.addMemSection( app_log_mem        );
mpu1_mmap.checkOverlap();

sys_mmap = MemoryMap("System Memory Map");
sys_mmap.addMemSection( mpu1_msmc          );
sys_mmap.addMemSection( mpu1_ddr           );
sys_mmap.addMemSection( app_log_mem        );
sys_mmap.checkOverlap();

#
# Generate linker command files containing "MEMORY" definitions
#
LinkerCmdFile(mpu1_mmap  , "./mpu1/linker_mem_map.cmd"  ).export();

HtmlMmapTable(sys_mmap, "./system_memory_map.html").export();
