#
# NAND Flash Controller Device Driver
# Copyright (c) 2008-2011, Intel Corporation and its suppliers.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#
#

# Makefile for linux spectra driver
# Will be executed twice:
#  - from user issuing make
#  - from the linux kernel build process
# KERNELRELEASE is defined ONLY when this makefile is part of a kenel
# build.
#
# $Id$

obj-$(CONFIG_SPECTRA_NAND_FLASH) := spectra.o
spectra-objs := ffsport.o lld_esl.o lld.o lld_emu.o lld_cdma.o lld_nand.o flash.o flash_bbt.o
EXTRA_CFLAGS := -DLINUX_DEVICE_DRIVER
