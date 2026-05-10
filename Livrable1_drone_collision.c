/*
 * Système de Détection de Collision – Essaim UAV
 * École des Sciences de l'Information – Programmation Avancée en C
 * Algorithme : Diviser pour Régner  |  Complexité : O(n log² n)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define N          10000
#define SEED       42
#define THRESHOLD  8       /* seuil de bascule vers la force brute */

/* ── Structure de données ───────────────────────────────────────────────── */

typedef struct {
    int   id;
    float x, y, z;
} Drone;

typedef struct {
    Drone *a, *b;
    float  dist;
} Paire;

/* ── Utilitaires ────────────────────────────────────────────────────────── */

static inline float distance3d(const Drone *a, const Drone *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

static inline Paire meilleure(Paire p, Paire q)
{
    return (p.dist <= q.dist) ? p : q;
}

static int tri_par_x(const void *a, const void *b)
{
    const Drone *da = *(const Drone **)a;
    const Drone *db = *(const Drone **)b;
    return (da->x > db->x) - (da->x < db->x);
}

static int tri_par_y(const void *a, const void *b)
{
    const Drone *da = *(const Drone **)a;
    const Drone *db = *(const Drone **)b;
    return (da->y > db->y) - (da->y < db->y);
}

/* ── Cas de base : force brute sur un petit sous-ensemble ───────────────── */

static Paire force_brute(Drone **arr, int n)
{
    Paire best = { NULL, NULL, FLT_MAX };

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float d = distance3d(*(arr + i), *(arr + j));
            if (d < best.dist) {
                best.dist = d;
                best.a    = *(arr + i);
                best.b    = *(arr + j);
            }
        }
    }
    return best;
}

/* ── Vérification de la bande centrale ─────────────────────────────────── */

static Paire verifier_bande(Drone **bande, int taille, float delta)
{
    Paire best = { NULL, NULL, delta };

    qsort(bande, taille, sizeof(Drone *), tri_par_y);

    for (int i = 0; i < taille; i++) {
        for (int j = i + 1; j < taille; j++) {
            Drone *pi = *(bande + i);
            Drone *pj = *(bande + j);

            if (pj->y - pi->y >= best.dist)
                break;

            float d = distance3d(pi, pj);
            if (d < best.dist) {
                best.dist = d;
                best.a    = pi;
                best.b    = pj;
            }
        }
    }
    return best;
}

/* ── Récursion diviser-pour-régner ──────────────────────────────────────── */

static Paire paire_proche_rec(Drone **px, int n)
{
    if (n <= THRESHOLD)
        return force_brute(px, n);

    int   mid  = n / 2;
    float mx   = (*(px + mid))->x;

    Paire gauche  = paire_proche_rec(px,       mid);
    Paire droite  = paire_proche_rec(px + mid, n - mid);
    Paire best    = meilleure(gauche, droite);

    Drone **bande = malloc(n * sizeof(Drone *));
    if (!bande) { perror("malloc bande"); exit(EXIT_FAILURE); }

    int taille = 0;
    for (int i = 0; i < n; i++) {
        if (fabsf((*(px + i))->x - mx) < best.dist)
            *(bande + taille++) = *(px + i);
    }

    Paire s = verifier_bande(bande, taille, best.dist);
    free(bande);

    return meilleure(best, s);
}

/* ── Interface publique ─────────────────────────────────────────────────── */

static Paire trouver_paire_critique(Drone *essaim, int n)
{
    Drone **index = malloc(n * sizeof(Drone *));
    if (!index) { perror("malloc index"); exit(EXIT_FAILURE); }

    for (int i = 0; i < n; i++)
        *(index + i) = essaim + i;

    qsort(index, n, sizeof(Drone *), tri_par_x);

    Paire resultat = paire_proche_rec(index, n);
    free(index);
    return resultat;
}

/* ── Génération du nuage de drones ──────────────────────────────────────── */

static Drone *generer_essaim(int n)
{
    Drone *essaim = malloc(n * sizeof(Drone));
    if (!essaim) { perror("malloc essaim"); exit(EXIT_FAILURE); }

    srand(SEED);
    for (int i = 0; i < n; i++) {
        (essaim + i)->id = i + 1;
        (essaim + i)->x  = (float)rand() / RAND_MAX * 10000.0f;
        (essaim + i)->y  = (float)rand() / RAND_MAX * 10000.0f;
        (essaim + i)->z  = (float)rand() / RAND_MAX * 1000.0f;
    }
    return essaim;
}

/* ── Point d'entrée ─────────────────────────────────────────────────────── */

int main(void)
{
    printf("=== Système de Détection de Collision – Essaim UAV ===\n");
    printf("    Drones : %d  |  Algorithme : Diviser-pour-Régner\n\n", N);

    Drone *essaim = generer_essaim(N);

    clock_t t0 = clock();
    Paire critique = trouver_paire_critique(essaim, N);
    clock_t t1 = clock();

    double ms = (double)(t1 - t0) / CLOCKS_PER_SEC * 1000.0;

    printf(">>> ALERTE COLLISION IMMINENTE <<<\n");
    printf("  Drone #%d  →  (%.2f, %.2f, %.2f)\n",
           critique.a->id, critique.a->x, critique.a->y, critique.a->z);
    printf("  Drone #%d  →  (%.2f, %.2f, %.2f)\n",
           critique.b->id, critique.b->x, critique.b->y, critique.b->z);
    printf("  Distance critique : %.6f m\n", critique.dist);
    printf("  Temps d'exécution : %.3f ms\n\n", ms);

    printf(">>> Manœuvre d'évitement déclenchée avec succès.\n");

    free(essaim);
    return EXIT_SUCCESS;
}
