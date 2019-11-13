#ifndef BRON_KERBOSCH_H
#define BRON_KERBOSCH_H

#include <list>
#include <unordered_set>
#include <algorithm>
#include <functional>



namespace caterpillar::detail
{

template <typename T>
struct Vertex
{
    Vertex (const T id) : id {id}
    {
    }

    bool operator == (const Vertex & other) const
    {
        return this->id == other.id;
    }

    T id;
    std::unordered_set <T> ns;
};



template <typename T> using Graph  = std::list <Vertex <T> >;
template <typename T> using Clique = Graph <T>;



template <typename T>
void solve (Graph <T> R, Graph <T> P, Graph <T> X, std::function <void (Graph <T>, Graph <T>, Graph <T>)> act)
{
    if (P.empty () && X.empty () ) {
        act (R, P, X);
        return;
    }
    while (!P.empty () ) {
        auto Ri = R;
        auto Pi = P;
        auto Xi = X;
        const auto v = P.front ();
        Ri.push_front (v);
        Pi.remove_if ([& v](const Vertex <T> & p){return !v.ns.count (p.id);});
        Xi.remove_if ([& v](const Vertex <T> & x){return !v.ns.count (x.id);});
        solve <T> (Ri, Pi, Xi, act);
        P.remove (v);
        X.push_front (v);
    }
}

}//namespace BronKerbosch

#endif//BRON_KERBOSCH_H
