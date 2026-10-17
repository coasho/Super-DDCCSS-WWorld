// Training links on its own below the SDK assembly (ARCH-05; tests/CMakeLists.txt, training_link): this program is linked
// with every object of fsim_training_mission and fsim_env and only their declared closure. It runs nothing; the test is
// that the link resolves - training needs no symbol from the final SDK assembly that links it - and that the program
// loads.

int main() {
    return 0;
}
