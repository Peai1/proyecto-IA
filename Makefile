TARGET = AE

SRC = src/AE.cpp

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Ejecutar el programa
run: $(TARGET)
	./$(TARGET)

# Limpiar archivos generados
clean:
	rm -f $(TARGET)
