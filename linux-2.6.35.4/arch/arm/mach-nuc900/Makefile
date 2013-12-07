#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y				:= irq.o time.o mfp.o gpio.o clock.o
obj-y				+= clksel.o dev.o cpu.o dmac.o fmi.o
# W90X900 CPU support files

obj-$(CONFIG_CPU_W90P910)	+= nuc910.o
obj-$(CONFIG_CPU_NUC950)	+= nuc950.o
obj-$(CONFIG_CPU_NUC951)	+= nuc951.o
obj-$(CONFIG_CPU_NUC952)	+= nuc952.o
obj-$(CONFIG_CPU_NUC945)	+= nuc945.o
obj-$(CONFIG_CPU_NUC946)	+= nuc946.o
obj-$(CONFIG_CPU_NUC960)	+= nuc960.o
obj-$(CONFIG_CPU_NUC920)	+= nuc920.o

# machine support

obj-$(CONFIG_MACH_W90P910EVB)	+= mach-nuc910evb.o
obj-$(CONFIG_MACH_W90P950EVB)	+= mach-nuc950evb.o
obj-$(CONFIG_MACH_W90P951EVB)	+= mach-nuc951evb.o
obj-$(CONFIG_MACH_W90P952EVB)	+= mach-nuc952evb.o
obj-$(CONFIG_MACH_W90P945EVB)	+= mach-nuc945evb.o
obj-$(CONFIG_MACH_W90P946EVB)	+= mach-nuc946evb.o
obj-$(CONFIG_MACH_W90N960EVB)	+= mach-nuc960evb.o
obj-$(CONFIG_MACH_W90P920EVB)	+= mach-nuc920evb.o

# Add pci support for nuc960, nuc920
obj-$(CONFIG_PCI)			+= pci.o
