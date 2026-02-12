################################################################################
# Makefile for ADuC841 RX Decoder Project
# Target: Keil µVision C51 Compiler
################################################################################

# Project name
PROJECT = rx_decoder

# Keil C51 tools (adjust paths to your installation)
CC = C51
LINK = BL51
HEX = OH51

# Compiler flags
CFLAGS = --model-small \
         --opt-code-speed \
         --debug

# Target chip
TARGET = ADuC841

# Source files
SOURCES = main.c \
          rx_decoder.c

# Object files
OBJECTS = $(SOURCES:.c=.obj)

# Build targets
all: $(PROJECT).hex

# Compile C files to object files
%.obj: %.c
	$(CC) $< $(CFLAGS)

# Link object files
$(PROJECT).abs: $(OBJECTS)
	$(LINK) $(OBJECTS) TO $(PROJECT)

# Generate Intel HEX file
$(PROJECT).hex: $(PROJECT).abs
	$(HEX) $(PROJECT)

# Clean build artifacts
clean:
	rm -f *.obj *.lst *.abs *.hex *.bin *.map *.lnp

# Flash to target (requires flasher tool)
flash: $(PROJECT).hex
	@echo "Flash $(PROJECT).hex to ADuC841"
	# Add your flash tool command here
	# Example: aduc_flash -p $(PROJECT).hex

################################################################################
# Alternative: Command-line build instructions for Keil µVision IDE
################################################################################

# For Keil µVision IDE users:
# 1. Create new project: Project -> New Project
# 2. Select device: ADuC841 (or compatible 8052)
# 3. Add files: 
#    - Right-click "Source Group 1" -> Add Files
#    - Add: main.c, rx_decoder.c
# 4. Configure project options:
#    Target -> Code Generation -> Memory Model: Small
#    Output -> Create HEX File: Enabled
# 5. Build: Project -> Build Target (F7)

################################################################################
# SDCC Alternative (Open Source Compiler)
################################################################################

# If using SDCC instead of Keil C51:
CC_SDCC = sdcc
CFLAGS_SDCC = -mmcs51 \
              --model-small \
              --stack-auto \
              --opt-code-speed

sdcc: $(SOURCES)
	$(CC_SDCC) $(CFLAGS_SDCC) $(SOURCES) -o $(PROJECT).ihx
	@echo "SDCC build complete: $(PROJECT).ihx"

################################################################################
# Testing (PC build)
################################################################################

test: test_decoder.c rx_decoder.h
	gcc test_decoder.c -o test_decoder -Wall -Wextra -std=c99
	./test_decoder
	@echo "All tests passed!"

################################################################################
# Documentation
################################################################################

docs:
	@echo "Documentation files:"
	@echo "  README.md          - Project overview"
	@echo "  INTEGRATION_GUIDE.md - Integration examples"
	@echo "  rx_decoder.h       - API reference"

################################################################################
# Project structure
################################################################################

tree:
	@echo "Project Structure:"
	@echo "."
	@echo "├── Makefile"
	@echo "├── README.md"
	@echo "├── INTEGRATION_GUIDE.md"
	@echo "├── main.c              (example integration)"
	@echo "├── rx_decoder.c        (decoder implementation)"
	@echo "├── rx_decoder.h        (API header)"
	@echo "└── test_decoder.c      (PC-based unit tests)"

################################################################################
# Help
################################################################################

help:
	@echo "ADuC841 RX Decoder Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all       Build project (Keil C51)"
	@echo "  sdcc      Build with SDCC (open source)"
	@echo "  test      Run unit tests on PC"
	@echo "  clean     Remove build artifacts"
	@echo "  flash     Flash to ADuC841"
	@echo "  docs      Show documentation"
	@echo "  tree      Show project structure"
	@echo "  help      Show this message"
	@echo ""
	@echo "For Keil µVision IDE, see comments in Makefile"

.PHONY: all clean flash sdcc test docs tree help
