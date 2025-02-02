TARGET = AE

SRC = src/AE.cpp

CXX = g++
CXXFLAGS = -std=c++17

INSTANCIA ?= instancias/AB101.dat
ITERACIONES ?= 20

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Ejecutar el programa
run: $(TARGET)
	./$(TARGET) $(INSTANCIA) $(ITERACIONES)

# Limpiar archivos generados
clean:
	rm -f $(TARGET)
