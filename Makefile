INC_DIR := inc
SRC_DIR := src
BIN_DIR := bin
LIB_DIR = libs

CC ?= gcc
CFLAGS := -Wall -g
CPPFLAGS := -I$(INC_DIR) -MMD -MP
LFLAGS =
LIBS =

SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ = $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
LIB = $(SRC:$(SRC_DIR)/%.c=$(LIB_DIR)/lib%.a)

# define the executable file
MAIN =test.c
EXEC =$(BIN_DIR)/test

####

.PHONY:	depend clean

all:	$(EXEC) $(LIB)

$(LIB): $(OBJ)
	@mkdir -p $(LIB_DIR)
	@$(AR) cr $@ $^
	@echo "Archive $(notdir $@)"

$(EXEC):	$(OBJ) | $(BIN_DIR)
	$(CC) -I$(INC_DIR) $(CFLAGS) $(MAIN) -o $(EXEC) $(OBJ) $(LFLAGS) $(LIBS)

$(BIN_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

clean:
	$(RM) -rv $(BIN_DIR) $(LIB_DIR)
