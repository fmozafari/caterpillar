#ifndef BRON_KERBOSCH_UTILS_H
#define BRON_KERBOSCH_UTILS_H

#include "bron_kerbosch.hpp"



namespace caterpillar::detail
{

using namespace std::placeholders;

template <typename T> using Solution = std::list <Graph <T> >;


template <typename T>
void aggregator (Solution <T> & solution, Graph <T> R, Graph <T> P, Graph <T>)
{
  solution.push_front (R);
  for (const auto & v : R) {
    P.remove (v);
  }
  if (!P.empty () ) {
    solve <T> ({ {} }, P, { {} }, std::bind (aggregator <T>, std::ref (solution), _1, _2, _3) );
  }
};

template <typename T>
bool edge (Graph <T> & G, const T & a, const T & b)
{
    auto ai = std::find_if (G.begin (), G.end (), [&](const Vertex <T> & v){return v.id == a;});
    auto bi = std::find_if (G.begin (), G.end (), [&](const Vertex <T> & v){return v.id == b;});
    if (G.end () != ai && G.end () != bi) {
        ai->ns.insert (b);
        bi->ns.insert (a);

        return true;
    }

    return false;
};



template <typename T>
Graph <T> complement (const Graph <T> & G)
{
    Graph <T> N = G;

    std::list <T> a;
    for (const auto & v : G) {
        a.push_front (v.id);
    }

    for (auto & v : N) {
        const auto t = v.ns;
        v.ns.clear ();
        for (const auto & b : a) {
            if (v.id != b && t.end () == std::find (t.begin (), t.end (), b) ) {
                v.ns.insert (b);
            }
        }
    }

    return N;
}

}//namespace caterpillar::detail

#endif//BRON_KERBOSCH_UTILS_H