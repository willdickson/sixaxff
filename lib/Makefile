# ---------------------------------------------------------------------
#  sixaxff 
#  Copyright (C) William Dickson, 2008.
#  
#  wbd@caltech.edu
#  www.willdickson.com
#
#  Released under the LGPL Licence, Version 3
#  
#  This file is part of sixaxff.
#
#  yawff is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#    
#  yawff is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
#  Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with yawff.  If not, see
#  <http://www.gnu.org/licenses/>.
#
# ----------------------------------------------------------------------
#  Makefile
#
#  Purpose: to build yawff library
#
#  Author: Will Dickson
# ---------------------------------------------------------------------- 
LIBRARY_NAME=sixaxff
MINOR_NUM=1
RELEASE_NUM=0.1
INSTALL_DIR=/usr/local/lib
ATIDAQ_SRC_DIR=../atidaq/src
ATIDAQ_INC_DIR=../atidaq/include/

vpath %.c ./
vpath %.c $(ATIDAQ_SRC_DIR) 
vpath %.h ./
vpath %.h $(ATIDAQ_INC_DIR) 

SRC=${LIBRARY_NAME}.c  util.c test-util.c sixax.c\
		ftconfig.c \
		ftrt.c \
		dom.c \
		expatls.c \
	 	node.c \
		stack.c \
		xmlparse.c \
		xmlrole.c \
		xmltok.c

OBJ=${SRC:.c=.o} 
TARGET=lib${LIBRARY_NAME}.so.${MINOR_NUM}

CC= $(shell rtai-config --cc)
LXRT_CFLAGS= $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS= $(shell rtai-config --lxrt-ldflags)
TEST_LDFLAGS= -lcunit
#INCLUDES= -I  
LOCAL_INCLUDES= -iquote${ATIDAQ_INC_DIR}
CFLAGS= -fPIC -c
CSTANDARD= -std=gnu89
DARRICK=-D ARRICK

all: $(TARGET) test devel

$(TARGET): $(OBJ)
	${CC} -shared -W1,-soname,$@ -o $@.$(RELEASE_NUM) $? -lm $(LXRT_LDFLAGS) 

$(OBJ): $(SRC)	
	$(CC) $(CSTANDARD) $(CFLAGS) $(LOCAL_INCLUDES) $(LXRT_CFLAGS) $(DARRICK) $?  

test: $(SRC) test-$(LIBRARY_NAME).c
	$(CC) $(CSTANDARD) $(LOCAL_INCLUDES) $(LXRT_CFLAGS) $(DARRICK) $? -o test-$(LIBRARY_NAME) $(TEST_LDFLAGS) -lm $(LXRT_LDFLAGS)

devel: $(SRC) devel-$(LIBRARY_NAME).c
	$(CC) $(CSTANDARD) $(LOCAL_INCLUDES) $(LXRT_CFLAGS) $(DARRICK) $? -o devel-$(LIBRARY_NAME) $(TEST_LDFLAGS) -lm $(LXRT_LDFLAGS)

.PHONY: clean install uninstall

clean: 
	if [ -f $(TARGET).$(RELEASE_NUM) ]; then rm $(TARGET).$(RELEASE_NUM); fi
	if [ -f test-$(LIBRARY_NAME) ]; then rm test-$(LIBRARY_NAME); fi
	if [ -f devel-$(LIBRARY_NAME) ]; then rm devel-$(LIBRARY_NAME); fi
	-rm *.o
	-rm *~

install:
	cp $(TARGET).$(RELEASE_NUM) $(INSTALL_DIR)
	if [ ! -h $(INSTALL_DIR)/$(TARGET) ]; then ln -s ${INSTALL_DIR}/$(TARGET).$(RELEASE_NUM) ${INSTALL_DIR}/$(TARGET); fi
	ldconfig

uninstall:
	rm ${INSTALL_DIR}/$(TARGET)
	rm ${INSTALL_DIR}/$(TARGET).$(RELEASE_NUM)
	ldconfig	
