/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/

#pragma once
#include <boost/dynamic_bitset.hpp>
#include "mapping_strategy.hpp"
#include <caterpillar/solvers/solver_manager.hpp>
#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/structures/pebbling_view.hpp>
#include <caterpillar/structures/abstract_network.hpp>
#include <caterpillar/synthesis/strategies/pebbling_mapping_strategy.hpp>
#include <caterpillar/solvers/z3_solver.hpp>
#include <mockturtle/networks/abstract_xag.hpp>
#include <mockturtle/views/topo_view.hpp>
#include <mockturtle/views/depth_view.hpp>
#include <tweedledum/networks/netlist.hpp>
#include <caterpillar/details/bron_kerbosch.hpp>
#include <caterpillar/details/bron_kerbosch_utils.hpp>
#include <caterpillar/details/depth_costs.hpp>
#include <algorithm>
#include <chrono>
#ifdef USE_iGRAPH
#include <igraph.h>
#endif
#include <fmt/format.h>
using namespace std::chrono;


namespace caterpillar
{
using abstract_xag_network = mockturtle::abstract_xag_network;
using abs_node_t = abstract_xag_network::node;
using abs_steps_xag_t = std::vector<std::pair<abs_node_t, mapping_strategy_action>>;


inline std::vector<cone_t> get_cones( abs_node_t node, abstract_xag_network const& xag, std::vector<uint32_t> const& drivers )
{
  assert(xag.is_and(node));
  // I can assume that AND inputs are not complemented
  // AND I can assume that include cases have been taken care of
  // collect the cones
  std::vector<cone_t> cones;
  xag.foreach_fanin(node, [&] (auto f)
  {
    std::vector<uint32_t> lsets;
    auto fi = xag.get_node(f);
    if(xag.is_nary_xor(fi) && std::find(drivers.begin(), drivers.end(), fi) == drivers.end())
    {
      xag.foreach_fanin(fi, [&] (auto s)
      {
        auto l = xag.get_node(s);
        lsets.push_back(l);
      });
    }
    else 
    {
      lsets = {fi};
    }
    assert(!lsets.empty());
    if(lsets.size() == 1) assert(lsets[0] == fi);
    cones.push_back({fi, lsets});
  });

  assert(cones.size() == 2);

  auto left = cones[0].leaves;
  auto right = cones[1].leaves;


  /* search a target for first */
  std::set_difference(left.begin(), left.end(), 
    right.begin(), right.end(), std::inserter(cones[0].target, cones[0].target.begin()));

  /* set difference */
  std::set_difference(right.begin(), right.end(), 
    left.begin(), left.end(), std::inserter(cones[1].target, cones[1].target.begin()));
  
  return cones;
}

static inline abs_steps_xag_t gen_steps( abs_node_t node, bool compute, abstract_xag_network const& xag, std::vector<uint32_t> const& drivers)
{
  abs_steps_xag_t parity_steps;
  abs_steps_xag_t comp_steps;

  if(xag.is_and(node))
  {
    auto cones = get_cones(node, xag, drivers);
    // compute inplace xors
    for(auto cone : cones)
    {
      if(xag.is_and(cone.root) || xag.is_pi(cone.root) || cone.leaves.size() == 1) continue;
      if ( cone.target.empty() )
      {
        parity_steps.push_back( {cone.root, compute_action{ cone.leaves, std::nullopt}} );
      }
      else 
      {
        parity_steps.push_back( {cone.root, compute_inplace_action{static_cast<uint32_t>( cone.target[0] ), cone.leaves}} );
      } 
    }

    comp_steps.insert(comp_steps.begin(), parity_steps.begin(), parity_steps.end());

  }

  if(compute)
    comp_steps.push_back( {node, compute_action{}} );
  else
    comp_steps.push_back( {node, uncompute_action{}} );

  if (xag.is_and(node)) 
    comp_steps.insert(comp_steps.end(), parity_steps.begin(), parity_steps.end());
  
  return comp_steps;
}

/* does not clean up possibly empty levels */
static inline std::vector<std::vector<abs_node_t>> get_levels_asap( abstract_xag_network const& xag_t, std::vector<abs_node_t> const& drivers)
{
  depth_view<abstract_xag_network, caterpillar::xag_depth_cost<abstract_xag_network>> xag {xag_t};

  /* AND nodes per level */
  std::vector<std::vector<uint32_t>> nodes_per_level( xag.depth() );
  xag.foreach_gate( [&]( auto const& n ) {
    if( xag.is_and(n) || std::find( drivers.begin(), drivers.end(), n ) != drivers.end() ) 
    {
      auto l = xag.level(n);
      nodes_per_level[l - 1u].push_back( n );
    }
  });
  return nodes_per_level;
}

/* get nodes per level (ALAP) */
static inline std::vector<std::vector<abs_node_t>> get_levels_alap( abstract_xag_network const& xag_t, std::vector<abs_node_t> const& drivers)
{
  mockturtle::depth_view<abstract_xag_network, xag_depth_cost<abstract_xag_network>> xag {xag_t};

  /* AND nodes per level */
  std::vector<std::vector<uint32_t>> nodes_per_level( xag.depth() );

  /* compute parents */
  node_map<std::vector<uint32_t>, abstract_xag_network> parents( xag );
  xag.foreach_node( [&]( auto const& n ) {
    if ( !( xag.is_and( n ) || (std::find(drivers.begin(), drivers.end(), n) != drivers.end()) ) ) return;

    xag.foreach_fanin( n, [&]( auto const& f ) {
      const auto child = xag.get_node( f );
      if ( xag.is_and( child ) )
      {
        parents[child].push_back( n );
      }
      else if ( xag.is_nary_xor( child ) )
      {
        xag.foreach_fanin( child, [&]( auto const& gf ) {
          const auto gchild = xag.get_node( gf );
          if ( xag.is_and( gchild ) )
          {
            parents[gchild].push_back( n );
          }
        });
      }
    });
  } );

  /* reverse TOPO */
  node_map<uint32_t, abstract_xag_network> level( xag );
  for ( auto n = xag.size() - 1u; n > 0u; --n )
  {
    if ( !( xag.is_and( n ) || (std::find(drivers.begin(), drivers.end(), n) != drivers.end()) ) ) continue;

    if ( parents[n].empty() )
    {
      const auto l = xag.depth() - 1u;
      nodes_per_level[l].push_back( n );
      level[n] = l;
    }
    else
    {
      const auto l = level[*std::min_element( parents[n].begin(), parents[n].end(), [&]( auto n1, auto n2 ) { return level[n1] < level[n2]; } )] - 1u;
      nodes_per_level[l].push_back( n );
      level[n] = l;
    }
  }

  return nodes_per_level;
}

/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It exploits two main facts:

    1.  XORs are relatively cheap to be implemented in fault tolerant quantum computing
    2.  Toffoli gates visited to implement AND nodes can be uncomputed using 0 T gates

    Details can be found in :cite:`MSC19`.
  \endverbatim
*/
class abstract_xag_mapping_strategy : public mapping_strategy<abstract_xag_network>
{

public:
  bool compute_steps( abstract_xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);                                                     
    //auto fi  = get_fi(xag, drivers);
    auto it = steps().begin();

    xag.foreach_gate( [&]( auto node ) {
      
      if ( xag.is_and( node ) || std::find( drivers.begin(), drivers.end(), node ) != drivers.end() )
      {

        auto cc = gen_steps( node , /* compute */ true, xag, drivers);
        it = steps().insert( it, cc.begin(), cc.end() );
        it = it + cc.size();

        if ( std::find( drivers.begin(), drivers.end(), node ) == drivers.end()  )
        { 
          auto uc = gen_steps( node , false, xag, drivers);
          it = steps().insert( it, uc.begin(), uc.end() );
        }

      }

    } );

    return true;
  }
};

class abstract_xag_fast_lowt_mapping_strategy : public mapping_strategy<abstract_xag_network>
{

public:
  bool compute_steps( abstract_xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);                                                     
    auto levels = get_levels_asap(xag, drivers);

    auto it = steps().begin();

    for(auto lvl : levels)
    { 
      assert( lvl.size() > 0 );
      /* store two action sets for each node in the level */
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> node_and_action;      
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> to_be_uncomputed;

      for(auto n : lvl)
      {
        /* nary xor outs have empty cones */
        if(xag.is_and(n))
        {
          auto cones = get_cones(n, xag, drivers);
          node_and_action.push_back({n, cones });
        }
        else
        {
          node_and_action.push_back({n, {}});
        }
      }

      it = steps().insert(it, {lvl[0], compute_level_action{node_and_action}});
      it = it + 1;
      
      for(auto node : node_and_action)
      {
        if(std::find(drivers.begin(), drivers.end(), node.first) == drivers.end())
        to_be_uncomputed.push_back(node);
      }
      it = steps().insert(it, {lvl[0], uncompute_level_action{to_be_uncomputed}});
    }

    return true;
  }
};
/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It finds a strategy that aim to minimize the number of T-stages or T-depth.
    Every level uses some extra-qubits to copy AND inputs.
  \endverbatim
*/
class abstract_xag_low_depth_mapping_strategy : public mapping_strategy<abstract_xag_network>
{

  void eval_copies(std::vector<cone_t>& cones, boost::dynamic_bitset<>& visited)
  {
    //problem gere as leaves are empty when there is a direct ling
    for(auto& cone : cones) 
    {
      for (auto l : cone.leaves )
      {
        assert( l < visited.size());
        if( visited[l] == true)
        {
          cone.copies.push_back(l);
        }
      }
    }
    for(auto cone : cones)
    {
      for(auto l : cone.leaves )
      {
        visited.set(l);
      }
    }
  }

  bool _alap;

public: 
  
  abstract_xag_low_depth_mapping_strategy (bool use_alap = false)
  : _alap(use_alap){}

  bool compute_steps( abstract_xag_network const& ntk ) override
  {
    // the strategy proceeds in topological order and level by level
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);
    auto levels = _alap ? get_levels_alap(xag, drivers) : get_levels_asap(xag, drivers);

    auto it = steps().begin();

    for(auto lvl : levels)
    {
      assert( lvl.size() > 0 );
      /* store two action sets for each node in the level */
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> node_and_action;      
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> to_be_uncomputed;


      boost::dynamic_bitset<> visited (xag.size());
      for(auto n : lvl)
      {
        if(xag.is_and(n))
        {
          auto cones = get_cones(n, xag, drivers);
          if(xag.is_and(n))
          {
            /* modifies cones.leaves and cones.copies according to visited */
            eval_copies(cones, visited);
          }
          node_and_action.push_back({n, cones});
        }
        else
        {
          node_and_action.push_back({n, {}});
        }
      }

      it = steps().insert(it, {lvl[0], compute_level_action{node_and_action}});
      it = it + 1;
      
      for(auto node : node_and_action)
      {
        if(std::find(drivers.begin(), drivers.end(), node.first) == drivers.end())
        to_be_uncomputed.push_back(node);
      }
      it = steps().insert(it, {lvl[0], uncompute_level_action{to_be_uncomputed}});
      
    }
    return true;
  }
};

#ifdef USE_iGRAPH
/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It finds a strategy that aim to minimize the number of T-stages or T-depth.
    It performs as many AND operations in parallel as possible.
  \endverbatim
*/

class abstract_xag_depth_fit_mapping_strategy : public mapping_strategy<abstract_xag_network>
{

  std::vector<boost::dynamic_bitset<>> get_mask( 
      std::vector<uint32_t> const& lvl, 
      abstract_xag_network const& xag, 
      std::vector<uint32_t> const& drivers)
  {
    std::vector<boost::dynamic_bitset<>> masks (lvl.size());
    for(uint32_t i = 0; i < lvl.size(); i++)
    {  
      masks[i].resize(xag.size());

      // nary xors will result compatible with any other node in the level
      if(xag.is_nary_xor(lvl[i])) continue;

      auto cones = get_cones(lvl[i], xag, drivers);  
      for (auto l : cones[0].leaves)
      {
        assert(masks[i].size() > l);
        masks[i].set(l);
      }
      for (auto l : cones[1].leaves)
      {
        assert(masks[i].size() > l);
        masks[i].set(l);
      }
    }
    return masks;
  }

  void build_compatibility_graph(
      igraph_t* g, 
      std::vector<uint32_t> const& lvl, 
      abstract_xag_network const& xag, 
      std::vector<uint32_t> const& drivers)
  {
    auto conflicts = get_mask(lvl, xag, drivers);

    /* build the graph comparing the masks */
    igraph_integer_t nv = lvl.size();
    igraph_vector_t edges;
    igraph_vector_init(&edges, 0);
    for(uint32_t i = 1; i < lvl.size(); i++)
    {
      for (uint32_t j = 0; j < i; j++ )
      {
        auto merge = conflicts[i] & conflicts[j];
        if ( merge.none())
        {
          igraph_vector_push_back(&edges , i);
          igraph_vector_push_back(&edges , j);
        }
      }
    }
    igraph_create( g, &edges, nv, /*undirected*/0);

    for( auto i = 0u; i < lvl.size(); i++)
    {
      SETVAN(g, "node", i, lvl[i]);
    }
  }

  std::vector<std::vector<uint32_t>> clique_cover( igraph_t* const graph)
  {
    std::vector<std::vector<uint32_t>> cover;
    igraph_vector_ptr_t cliques; 
    igraph_t _g; 
  
    igraph_vector_ptr_init(&cliques, 0);
    igraph_copy(&_g, graph);
    

    while(igraph_vcount(&_g) != 0)
    {
      igraph_vector_ptr_clear(&cliques);
      std::vector<uint32_t> clique;

      igraph_largest_cliques(&_g, &cliques);
      igraph_vector_t *v = (igraph_vector_t*)VECTOR(cliques)[0];
      for (int j = 0; j < igraph_vector_size(v); j++)
      {
        auto vid = VECTOR(*v)[j];
        auto node = VAN(&_g, "node", vid);
        clique.push_back(node);
      }
      cover.push_back(clique);

      /* removes the vertices of the largest clique       
      * it changes the ids but is fine if done at once
      * numerical vertex attributes should not be changed */
      igraph_delete_vertices(&_g, igraph_vss_vector(v));
      igraph_vector_destroy(v);
    }

    igraph_vector_ptr_destroy(&cliques);
    return cover;
  }

  void decompose_graph_cc(igraph_t* graph, igraph_vector_ptr_t* subgraphs, uint32_t size_limit)
  {

    igraph_vector_ptr_t components; igraph_vector_ptr_init(&components, 0);
    IGRAPH_VECTOR_PTR_SET_ITEM_DESTRUCTOR(&components, igraph_destroy);

    igraph_decompose(graph, &components, IGRAPH_WEAK, -1, 1);
    auto num_components = igraph_vector_ptr_size(&components);
    
    igraph_t* sub;
    for (int i = 0; i < num_components; i++) 
    {
      auto comp_size = igraph_vcount((igraph_t *)VECTOR(components)[i]);
      if(comp_size > size_limit)
      {
        auto from = 0;
        auto rem_size = comp_size;
        while (rem_size > size_limit)
        {
          auto to = from + size_limit - 1;
          sub = igraph_Calloc(1, igraph_t);
          igraph_induced_subgraph(graph, sub, igraph_vss_seq(from, to), IGRAPH_SUBGRAPH_CREATE_FROM_SCRATCH);
          igraph_vector_ptr_push_back(subgraphs, sub);
      
          rem_size = rem_size-size_limit;
          from = to + 1;
        }
        if(rem_size > 0)
        {
          sub = igraph_Calloc(1, igraph_t);
          igraph_induced_subgraph(graph, sub, igraph_vss_seq(from, from+rem_size-1), IGRAPH_SUBGRAPH_CREATE_FROM_SCRATCH);
          igraph_vector_ptr_push_back (subgraphs, sub);
        }

      }
      else
      {
        sub = igraph_Calloc(1, igraph_t);
        igraph_copy(sub, (igraph_t *)VECTOR(components)[i]);
        igraph_vector_ptr_push_back(subgraphs, sub );
      } 
    }
    igraph_vector_ptr_destroy_all(&components);
  }

  /* records the largest clique size that is the maximum amount of qubits added by a Tdepth=1 implementation 
  as smaller cliques can reuse it */
  uint32_t size_limit= std::numeric_limits<uint>::max();

public: 

  abstract_xag_depth_fit_mapping_strategy(uint32_t max_size_comp_graph = std::numeric_limits<uint>::max())
  : size_limit(max_size_comp_graph) {}

  bool compute_steps( abstract_xag_network const& ntk ) override
  {
    /* set up graph attribute handler */
    igraph_i_set_attribute_table(&igraph_cattribute_table);

    // the strategy proceeds in topological order
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);
    
    /* each m_level is filled with AND nodes and XOR outputs */
    /* fi is propagated */
    auto levels = get_levels_asap(xag, drivers);                              

    auto it = steps().begin();

    for(auto lvl : levels)
    {
      assert(lvl.size()!=0);

      igraph_t graph; 
      igraph_vector_ptr_t subgraphs; 
      igraph_vector_ptr_init(&subgraphs, 0);

      build_compatibility_graph(&graph, lvl, xag, drivers);
      
      decompose_graph_cc(&graph, &subgraphs, size_limit);

      for(auto i = 0 ; i < igraph_vector_ptr_size(&subgraphs); i++)
      {
        assert(igraph_vcount((igraph_t *)VECTOR(subgraphs)[i]) <= size_limit);
        auto cover = clique_cover((igraph_t *)VECTOR(subgraphs)[i]);
      
        for(auto clique : cover)
        {
          level_info_t to_be_computed;
          level_info_t to_be_uncomputed;

          for(auto n : clique)
          {
            if(xag.is_and(n))
              to_be_computed.push_back({n, get_cones(n, xag, drivers)});
            else
              to_be_computed.push_back({n, {}});
          }

          it = steps().insert(it, {clique[0], compute_level_action{to_be_computed}});
          it = it + 1;
        
          for(auto& n : clique)
          {
            if(std::find(drivers.begin(), drivers.end(), n) == drivers.end())
            {
              assert(xag.is_and(n));
              to_be_uncomputed.push_back({n, get_cones(n, xag, drivers)});
            }
          }

          it = steps().insert(it, {clique[0], uncompute_level_action{to_be_uncomputed}});
        }
      }

      IGRAPH_VECTOR_PTR_SET_ITEM_DESTRUCTOR(&subgraphs, igraph_destroy);
      igraph_vector_ptr_destroy_all(&subgraphs);
      igraph_destroy(&graph);
    }
    return true;
  }
};
#endif
} // namespace caterpillar