#
# Simcenter Amesim system make file
#


# This makefile has been created using the following cathegory path list
#	$AME
#	$AME/libsig
#	$AME/libmec
#	$AME/libhydr
#	$AME/libpn
#	$AME/libth
#	$AME/libemd
#	$AME/libesc
#	$AME/libeb
#	$AME/libmotion
#	$AME/libthh
# End category path list
# The MACHINETYPE variable can be used in -L statements
# or otherwise to separate machine dependent code

MACHINETYPE = win64-gcc

# Then the object files
OBJECTS = \
	c:/program\ files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFFD3.o \
	c:/program\ files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFTK0.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/UD00.o \
	c:/program\ files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFVORF0.o \
	c:/program\ files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFVF000.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/CTRPID0.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/CONS00.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SSINK.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SIGRECEI0.o \
	c:/program\ files/simcenter/2404/amesim/libmec/submodels/win64-gcc/PMV00.o \
	c:/program\ files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SIGTRANS0.o \
	c:/program\ files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFPUC0.o \
	C:/Program\ Files/Simcenter/2404/Amesim/submodels/win64-gcc/INTVARSENSOR0.o

OBJECTS2 = \
	"c:/program files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFFD3.o" \
	"c:/program files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFTK0.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/UD00.o" \
	"c:/program files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFVORF0.o" \
	"c:/program files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFVF000.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/CTRPID0.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/CONS00.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SSINK.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SIGRECEI0.o" \
	"c:/program files/simcenter/2404/amesim/libmec/submodels/win64-gcc/PMV00.o" \
	"c:/program files/simcenter/2404/amesim/libsig/submodels/win64-gcc/SIGTRANS0.o" \
	"c:/program files/simcenter/2404/amesim/libthh/submodels/win64-gcc/TFPUC0.o" \
	"C:/Program Files/Simcenter/2404/Amesim/submodels/win64-gcc/INTVARSENSOR0.o"

Lemuel_pump_system_FMU_.dll: $(OBJECTS) Lemuel_pump_system_FMU_.o
	@echo Lemuel_pump_system_FMU_.make.link_args =
	@type Lemuel_pump_system_FMU_.make.link_args
	$(CC) -m64 -Wl,-E -shared $(LDFLAGS) -o Lemuel_pump_system_FMU_.dll Lemuel_pump_system_FMU_.o @"Lemuel_pump_system_FMU_.make.link_args" $(AMELIBS)

Lemuel_pump_system_FMU_.o: Lemuel_pump_system_FMU_.c
	$(CC) -c -m64 -I"$(AME)\interfaces\fmi" -I"$(AME)\interfaces" -I"$(AME)\interfaces\user_cosim" $(CFLAGS) -o Lemuel_pump_system_FMU_.o Lemuel_pump_system_FMU_.c

.c.o:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

.f.o:
	@echo
	@echo "Warning: \"$<\" is newer than the object."
	@echo ""

# End of file

