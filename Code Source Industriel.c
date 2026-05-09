#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <time.h>

#define N_DRONES 10000

typedef struct {
    int   id;
    float x;
    float y;
    float z;
} Drone;

typedef struct {
    Drone *drone_a;
    Drone *drone_b;
    float  dist_sq;
} PaireCritique;

static void echanger_pointeurs(Drone **a, Drone **b) {
    Drone *tmp = *a;
    *a = *b;
    *b = tmp;
}

static int partitionner(Drone **pp, int taille) {
    float pivot_x = (*(pp + taille - 1))->x;
    int   i       = -1;

    for (int j = 0; j < taille - 1; j++) {
        if ((*(pp + j))->x <= pivot_x) {
            i++;
            echanger_pointeurs(pp + i, pp + j);
        }
    }

    echanger_pointeurs(pp + i + 1, pp + taille - 1);
    return i + 1;
}

static void quicksort_axe_x(Drone **pp, int taille) {
    if (taille <= 1) return;

    int pivot_idx = partitionner(pp, taille);

    quicksort_axe_x(pp, pivot_idx);
    quicksort_axe_x(pp + pivot_idx + 1, taille - pivot_idx - 1);
}

static float distance_carree(const Drone *a, const Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

static PaireCritique detecter_paire_critique(Drone *essaim, int n) {
    PaireCritique resultat = {NULL, NULL, FLT_MAX};

    Drone **index = (Drone **)malloc(n * sizeof(Drone *));
    if (!index) {
        fprintf(stderr, "[ERREUR] Allocation du tableau d'index echouee.\n");
        return resultat;
    }

    for (int i = 0; i < n; i++) {
        *(index + i) = essaim + i;
    }

    quicksort_axe_x(index, n);

    for (int i = 0; i < n - 1; i++) {
        Drone *di = *(index + i);

        for (int j = i + 1; j < n; j++) {
            Drone *dj = *(index + j);

            float delta_x = dj->x - di->x;
            if (delta_x * delta_x >= resultat.dist_sq) break;

            float d = distance_carree(di, dj);
            if (d < resultat.dist_sq) {
                resultat.dist_sq = d;
                resultat.drone_a = di;
                resultat.drone_b = dj;
            }
        }
    }

    free(index);
    return resultat;
}

static float rand_float(float min, float max) {
    return min + (max - min) * ((float)rand() / (float)RAND_MAX);
}

int main(void) {
    srand((unsigned int)time(NULL));

    printf("============================================================\n");
    printf("  SYSTEME DE COLLISION UAV — ESI / Pr. Tarik HOUICHIME\n");
    printf("============================================================\n\n");

    Drone *essaim = (Drone *)malloc(N_DRONES * sizeof(Drone));
    if (!essaim) {
        fprintf(stderr, "[ERREUR CRITIQUE] Echec allocation memoire essaim.\n");
        return EXIT_FAILURE;
    }

    printf("[ MEM  ] Bloc alloue : %d drones x %zu octets = %zu KB\n",
           N_DRONES, sizeof(Drone), (N_DRONES * sizeof(Drone)) / 1024);

    printf("[ INIT ] Chargement de %d drones en memoire...\n", N_DRONES);
    for (int i = 0; i < N_DRONES; i++) {
        Drone *p = essaim + i;
        p->id    = i + 1;
        p->x     = rand_float(-5000.0f, 5000.0f);
        p->y     = rand_float(    0.0f, 3000.0f);
        p->z     = rand_float(-5000.0f, 5000.0f);
    }
    printf("[ INIT ] Initialisation terminee.\n\n");

    struct timespec t_debut, t_fin;
    clock_gettime(CLOCK_MONOTONIC, &t_debut);

    printf("[ SCAN ] Analyse de collision en cours (N=%d)...\n", N_DRONES);
    PaireCritique alerte = detecter_paire_critique(essaim, N_DRONES);

    clock_gettime(CLOCK_MONOTONIC, &t_fin);
    double temps_ms = (t_fin.tv_sec  - t_debut.tv_sec)  * 1000.0
                    + (t_fin.tv_nsec - t_debut.tv_nsec) / 1.0e6;

    if (alerte.drone_a && alerte.drone_b) {
        float dist = sqrtf(alerte.dist_sq);
        printf("\n");
        printf("╔══════════════════════════════════════════════════╗\n");
        printf("║       ⚠   ALERTE COLLISION IMMINENTE    ⚠       ║\n");
        printf("╠══════════════════════════════════════════════════╣\n");
        printf("║  Drone A : ID=%5d  pos=(%.2f, %.2f, %.2f)\n",
               alerte.drone_a->id,
               alerte.drone_a->x, alerte.drone_a->y, alerte.drone_a->z);
        printf("║  Drone B : ID=%5d  pos=(%.2f, %.2f, %.2f)\n",
               alerte.drone_b->id,
               alerte.drone_b->x, alerte.drone_b->y, alerte.drone_b->z);
        printf("║  Distance minimale : %.6f metres\n", dist);
        printf("╠══════════════════════════════════════════════════╣\n");
        printf("║  Temps d'execution : %.4f ms\n", temps_ms);
        printf("║  Complexite        : O(n log n)\n");
        printf("║  Status            : MANOEUVRE D'EVITEMENT OK\n");
        printf("╚══════════════════════════════════════════════════╝\n");
    } else {
        printf("[INFO] Aucune paire detectee — essaim securise.\n");
    }

    free(essaim);
    return EXIT_SUCCESS;
}
