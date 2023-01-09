BUILD_AS = gpasm
BUILD_AS_FLAGS = -p p16f887 -r dec

.PHONY: all
all: $(subst .asm,.hex,$(wildcard *.asm))

%.hex %.lst %.cod: %.asm
	$(BUILD_AS) $(BUILD_AS_FLAGS) -o $@ $^
