This is a tarball of the current Synaptics kernel.org touch device driver.
Included in the tarball are the full sources for the driver, along with some
related files.  Files in the tarball are:

      README-SYNAPTICS.txt
          This file.
      include/linux/rmi.h
          Public header file for the driver.
      drivers/input/rmi4/rmi*.[ch]
          Headers and source files for the driver.
      drivers/input/rmi4/Makefile
          Builds the driver.
      drivers/input/rmi4/Kconfig
          Config file
      drivers/input/Makefile
          For reference.
      drivers/input/Kconfig
          For reference.
      arch/arm/mach-omap2/board-omap3beagle.c
          A testing board file, for reference.
      arch/arm/configs/omap3_beagle_android_defconfig
          A testing kernel configuration, for reference.
      Documentation/input/rmi*.txt
          Associated documentation.



To apply the new driver codebase to your current system, please follow
the steps below:

+ This step is needed ONLY if you were previous using an older version of the
Synaptics driver that had sources in drivers/input/touchscreen.  In this case
you need to do the following before proceeding to the next step.
    * remove the files drivers/input/touchscreen/rmi*
    * edit drivers/input/touchscreen/Makefile and
drivers/input/touchscreen/KConfig to remove RMI driver references

+ copy the drivers/input/rmi4 folder from the tarball into your kernel tree

+ copy include/linux/rmi.h from the tarball into include/linux in your
kernel tree

+ edit drivers/input/Makefile and drivers/input/KConfig to reference
drivers/input/rmi4 appropriately (see the reference files in the tarball)

+ edit your defconfig file to remove TOUCHSCREEN_SYNAPTICS_RMI4_I2C and
related symbols; replace them with the appropriate CONFIG_RMI4_XXX
symbols.  See the omap3_beagle_android_defconfig file in the tarball for
reference.

+ update your board file to specify the appropriate platform data.  See the
attached board_omap3beagle.c file for reference.

+ do a make distclean to get rid of old object files and binaries

+ make defconfig to apply your configuration updates

+ finally, rebuild your kernel
