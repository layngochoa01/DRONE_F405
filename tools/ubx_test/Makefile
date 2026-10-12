# Harness PC cho parser UBX (biên dịch chính src/sensors/gps.c của firmware bằng gcc)
#   make            → build ./ubx_harness
#   make test       → chạy test đơn vị
#   make sample     → tạo sample.ubx rồi giải mã nó

ROOT   = ../..
CFLAGS = -std=c11 -Wall -Wextra -O1 -I$(ROOT)/include -I$(ROOT)/src/sensors -I$(ROOT)/src/drivers

ubx_harness: ubx_harness.c $(ROOT)/src/sensors/gps.c $(ROOT)/src/sensors/gps.h
	gcc $(CFLAGS) ubx_harness.c $(ROOT)/src/sensors/gps.c -lm -o $@

test: ubx_harness
	./ubx_harness --selftest

sample: ubx_harness
	./ubx_harness --make-sample sample.ubx
	./ubx_harness sample.ubx

clean:
	rm -f ubx_harness sample.ubx

.PHONY: test sample clean
