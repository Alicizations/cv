CC := g++
FLAGS := -std=c++11 -w -g
THE_FIRST := $<
ALL := $^
TARGET := $@
BUI_DIR := build

a: ./CImg.h ./sift.h ./siftpp/sift.cpp $(BUI_DIR)/ImageStitching.o $(BUI_DIR)/main.o
	$(CC) $(FLAGS) $^ -o $@ -O2 -lgdi32 -I ./ -I ./siftpp -lm -lpthread

$(BUI_DIR)/%.o: ./%.cpp ./%.hpp
	@mkdir -p $(BUI_DIR)
	$(CC) $(FLAGS) $< -o $@ -c

$(BUI_DIR)/main.o: ./main.cpp
	@mkdir -p $(BUI_DIR)
	$(CC) $(FLAGS) $< -o $@ -c

clean: 
	@rm -rf $(BUI_DIR)

s:
	g++ ./CImg.h ./sift.h ./siftpp/sift.cpp ./ImageStitching.cpp ./main.cpp -std=c++11 -w -g -o a -O2 -lgdi32 -I ./ -I ./siftpp -lm -lpthread

p:
	g++ ./CImg.h ./ps.cpp -std=c++11 -w -g -o b -O2 -lgdi32
