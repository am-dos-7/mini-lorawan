# cmac.out: mac_example.o
# 	cc -I /usr/include/openssl -o cmac.out mac_example.o -lssl -lcrypto

# mac_example.o: mac_example.c
# 	cc mac_example.c

lorawan_ns.out: lorawan_ns.c
	cc -I /usr/include/openssl -o lorawan_ns.out lorawan_ns.c -lssl -lcrypto

clean:
	rm lorawan_ns.out