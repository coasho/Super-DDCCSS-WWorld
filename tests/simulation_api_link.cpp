// The lower simulation facade links on its own (ARCH-04; tests/CMakeLists.txt, simulation_api_link): this program is
// linked with every object of fsim_simulation_api and only that target's declared closure. It runs nothing; the test is
// that the link resolves - the facade needs no symbol from training, graphics or the final SDK assembly - and that the
// program loads.

int main() {
    return 0;
}
