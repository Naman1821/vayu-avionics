/*
 * vayu_sim - run the firmware through simulated flights.
 *
 *   build/vayu_sim --list
 *   build/vayu_sim nominal            -> out/nominal/
 *   build/vayu_sim --all              -> out/<every scenario>/
 */
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "sim.h"

static int run(const scenario_t *sc)
{
    char dir[256];
    mkdir("out", 0755);
    snprintf(dir, sizeof dir, "out/%s", sc->name);
    printf("%-22s %s\n", sc->name, sc->about);
    fflush(stdout);
    return sim_run(sc, dir);
}

int main(int argc, char **argv)
{
    if (argc < 2 || strcmp(argv[1], "--list") == 0) {
        for (int i = 0; i < N_SCENARIOS; i++)
            printf("%-22s %s\n", SCENARIOS[i].name, SCENARIOS[i].about);
        return argc < 2;
    }
    if (strcmp(argv[1], "--all") == 0) {
        int rc = 0;
        for (int i = 0; i < N_SCENARIOS; i++)
            rc |= run(&SCENARIOS[i]);
        return rc;
    }
    for (int a = 1; a < argc; a++) {
        const scenario_t *sc = sim_find_scenario(argv[a]);
        if (!sc) {
            fprintf(stderr, "unknown scenario '%s' (try --list)\n", argv[a]);
            return 2;
        }
        if (run(sc))
            return 1;
    }
    return 0;
}
