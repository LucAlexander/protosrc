#!/bin/bash
for asm_file in *.src; do
	base_name=$(basename "$asm_file" .src)
	rom_file="${base_name}.rom"
	echo -e "\e[1;32m$base_name\e[0m"
	./orb -c "$asm_file" -o "$rom_file"
done
