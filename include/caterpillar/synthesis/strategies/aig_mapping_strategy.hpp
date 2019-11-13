/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/

#pragma once

#include "mapping_strategy.hpp"
#include "fmt/format.h"
#include <iostream>
#include <algorithm>

#include <tweedledum/networks/netlist.hpp>

#include <mockturtle/networks/aig.hpp>
#include <mockturtle/views/topo_view.hpp>
#include <mockturtle/views/depth_view.hpp>

#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/details/bron-kerbosch.hpp>
#include <caterpillar/details/bron-kerbosch_utils.hpp>


using namespace mockturtle;
using namespace std::placeholders;
using namespace caterpillar::detail;

using node_t = node<aig_network>;

namespace caterpillar
{
/*!
  \verbatim embed:rst
    This strategy is dedicated to AIG graphs and aims at minimizing circuit T-depth

    1.  Toffoli gates used to implement AND nodes can be uncomputed using 0 T gates
  \endverbatim
*/

struct aig_mapping_strategy_stats
{
  uint32_t t_depth = 0;

  uint32_t t_count = 0;

  uint32_t qubits = 0;
};

class aig_mapping_strategy : public mapping_strategy<mockturtle::aig_network>
{
  
  //structure containing the node of a aig and a bitset with bit i set if i is a child of the node
  struct node_and_children
  {
    node_and_children(){}

    node_t n;
    std::bitset<20> mask;

  };
  
  // gets for each level a vector of node_and_children
  std::vector<std::vector<node_and_children>> get_level_info (depth_view<aig_network> const& d_aig)
  {
    auto this_lvl = 0u; 

    std::vector<std::vector<node_and_children>> lvl_fi (d_aig.depth());

    d_aig.foreach_gate([&](auto const& node)
    {
      if( ! d_aig.is_pi(node) )
      {
        auto lvl = d_aig.level(node)-1;
        assert(this_lvl == this_lvl || this_lvl == lvl + 1);

        if ( lvl != this_lvl ) 
          this_lvl = lvl;

        node_and_children nc;
        nc.n = node;
        d_aig.foreach_fanin(node, [&](auto s)
        { 
          nc.mask.set(d_aig.get_node(s));
        });

        lvl_fi[this_lvl].push_back( nc );
      }
      
    });

    return lvl_fi;
  }

  //checks compatibility between nodes in one level and builds the graph
  detail::Graph <node_t> get_compatibility_graph( std::vector<node_and_children> const& lvl )
  {
    caterpillar::detail::Graph <node_t> P;

    for (auto vertex : lvl) 
    {
      P.push_front({vertex.n});
    }

    for(auto i = 0u; i < lvl.size(); i++)
    {
      for (auto j = i+1; j < lvl.size(); j++ )
      {
        auto current = lvl[i];
        auto other = lvl[j];
        if (current.n != other.n)
        { 

          auto merge = current.mask & other.mask;
          if ( merge.none())
          {
            caterpillar::detail::edge <node_t> (P, current.n, other.n);
          }
        }
      }
    }

    return P;
  }

  /* returns mapping steps for a clique
     the order of computation in the clique does not matter */
  std::vector<std::pair< node_t, mapping_strategy_action>> compute (Graph <node_t> const& clique, bool comp, aig_mapping_strategy_stats& st)
  {
    std::vector<std::pair<node_t, mapping_strategy_action>> cc;

    //every time a clique is computed t_depth is incremented
    if (comp) st.t_depth++;

    for (auto node : clique)
    {
      if(comp)
      {
        cc.push_back( {node.id, compute_action{}} );
        //every time an AND is computed we use 4 T gates and 1 qubit
        st.qubits++;
        st.t_count += 4;
      }
      else
      {
        cc.push_back( {node.id, uncompute_action{}} );
      }
      
    }
    return cc;
  }

  aig_mapping_strategy_stats* stp;

public:
  // constructor 
  aig_mapping_strategy( aig_mapping_strategy_stats* stp = nullptr)
    : stp(stp){}

  bool compute_steps( mockturtle::aig_network const& ntk ) override
  {
    //inits

    std::vector<mockturtle::node<mockturtle::aig_network>> drivers;
    ntk.foreach_po( [&]( auto const& f ) { drivers.push_back( ntk.get_node( f ) ); } );
    
    auto it = steps().begin();

    aig_mapping_strategy_stats st;

    st.qubits = ntk.num_pis();

    //topoview
    topo_view<aig_network> topo_aig (ntk);

    //depthview
    depth_view<aig_network> d_aig (topo_aig);

    //extract nodes and leaves for each level
    auto lvl_fi =  get_level_info (d_aig); 

    
    for(auto lvl : lvl_fi)
    {
      Graph <node_t> cgraph = get_compatibility_graph( lvl );

      // get the cliques
      caterpillar::detail::Solution<node_t> solution;
      caterpillar::detail::solve <node_t> ({ {} }, cgraph , { {} }, std::bind (caterpillar::detail::aggregator <node_t>, std::ref (solution), _1, _2, _3) );

      while( ! solution.empty() )
      {
        //decrescent order
        solution.sort( [&]  (const Graph <node_t>& first, const Graph <node_t>&  second) -> bool
        {
          return ( first.size() > second.size() );
        });


        auto clique = *solution.begin();

        auto cc = compute(clique, true, st);
  
        it = steps().insert(it, cc.begin(), cc.end());
        it = it+cc.size();

        if( (clique.size() != 1)  ||
            std::find (drivers.begin(), drivers.end(), clique.begin()->id ) == drivers.end() )
        {
          auto cu = compute(clique, false, st);
          it = steps().insert(it, cu.begin(), cu.end());
        }
          
        //remove clique from solution
        solution.remove(clique);

        for (auto v : clique)
        {
          for (auto& g : solution)
          {
            if (std::find(g.begin(), g.end(), v) != g.end()) 
            {
              g.remove(v);
            }
            
          }
        }
      }
    }

    if (stp) 
    {
      *stp = st;
    }

    return true;
  }
};

} // namespace caterpillar