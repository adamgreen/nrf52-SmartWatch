# User can set VERBOSE variable to have all commands echoed to console for debugging purposes.
ifdef VERBOSE
    Q :=
else
    Q := @
endif

# Useful macros
OBJS = $(patsubst ./%,$2/%,$(addsuffix .o,$(basename $(wildcard $1/*.c $1/*.m))))
MAKEDIR = mkdir -p $(dir $@)
REMOVE = rm
REMOVE_DIR = rm -r -f
QUIET = > /dev/null 2>&1 ; exit 0

# Tool flags
CLANG_FLAGS := -g -Wall -Werror -MMD -MP

# .o object files are to be placed in obj/ directory.
# .a lib files are to be placed in lib/ directory.
OBJDIR := obj

# Build this sample.
CONSOLE_APP := mriblue
CONSOLE_OBJ := $(call OBJS,.,$(OBJDIR))
DEPS += $(patsubst %.o,%.d,$(CONSOLE_OBJ))
FRAMEWORKS := -framework Foundation -framework AppKit -framework CoreBluetooth


# Rules
.PHONY : clean all

all : $(CONSOLE_APP)

$(CONSOLE_APP) : $(CONSOLE_OBJ)
	@echo Building $@
	$Q $(MAKEDIR) $(QUIET)
	$Q clang $(FRAMEWORKS) $^ -o $@
	@echo Signing $@
	sudo codesign --force --deep --sign - $@

clean :
	@echo Cleaning
	$Q $(REMOVE_DIR) $(OBJDIR) $(QUIET)
	$Q $(REMOVE_DIR) $(CONSOLE_APP)

# *** Pattern Rules ***
$(OBJDIR)/%.o : ./%.c
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q clang $(CLANG_FLAGS) -c $< -o $@

$(OBJDIR)/%.o : ./%.m
	@echo Compiling $<
	$Q $(MAKEDIR) $(QUIET)
	$Q clang $(CLANG_FLAGS) -c $< -o $@

# *** Pull in header dependencies if not performing a clean build. ***
ifneq "$(findstring clean,$(MAKECMDGOALS))" "clean"
    -include $(DEPS)
endif
