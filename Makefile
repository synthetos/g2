all: toolchain fw

.PHONY: toolchain
toolchain:
	@make -C Tools

.PHONY: fw
fw:
	@make -C TinyG2 PLATFORM=G2v9i

.PHONY: clean
clean:
	@make -C TinyG2 clean PLATFORM=G2v9i
