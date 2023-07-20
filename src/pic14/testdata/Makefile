BUILD_AS = gpasm
BUILD_AS_FLAGS = -p p16f887 -r dec

.PHONY: all
all: $(subst .asm,.hex,$(wildcard *.asm))

.PHONY: check
check:

.PHONY: clean
clean:
	rm -f $(subst .asm,.cod,$(wildcard *.asm))
	rm -f $(subst .asm,.lst,$(wildcard *.asm))
	rm -f $(subst .asm,.hex,$(wildcard *.asm))

%.cod: %.hex
%.lst: %.hex
%.hex: %.asm
	$(BUILD_AS) $(BUILD_AS_FLAGS) -o $@ $^
