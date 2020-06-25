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
#include <mockturtle/networks/xag.hpp>
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
using xag_network = mockturtle::xag_network;
using node_t = xag_network::node;
using steps_xag_t = std::vector<std::pair<node_t, mapping_strategy_action>>;
#ifdef USE_Z3
using steps_abs_t = std::vector<std::pair<abstract_network::node, mapping_strategy_action>>;
using SolverType = z3_pebble_solver<abstract_network>;
#endif


inline std::vector<uint32_t> sym_diff(std::vector<uint32_t> first, std::vector<uint32_t> second)
{
  std::vector<uint32_t> diff;
  /* this one works on sorted ranges */
  for(int i = 0; i< (int)(first.size()) - 1; i++)
    assert(first[i] < first[i+1]);
  
  for(int i = 0; i< (int)(second.size() - 1); i++)
    assert(second[i] < second[i+1]);
  
  std::set_symmetric_difference( first.begin(), first.end(), second.begin(), second.end(), std::back_inserter(diff) );
  return diff;
}

inline bool is_included(std::vector<uint32_t> first, std::vector<uint32_t> second)
{
  /* if second is included in first */
  bool is_included = true;
  for(auto l : first)
  {
    if (std::find( second.begin(), second.end(), l) == second.end())
    {
      is_included = false;
      break;
    }
  }
  return is_included;
}


inline void update_fi( node_t node, xag_network const& xag, std::vector<std::vector<uint32_t>>& fi, std::vector<node_t> const& drivers )
{

  if ( xag.is_and( node ) || xag.is_pi(node) || (std::find(drivers.begin(), drivers.end(), node) != drivers.end()))
  {
    fi[ xag.node_to_index(node) ] = { xag.node_to_index(node) };
  }

  else
  {      
    std::vector<uint32_t> fanin;
    xag.foreach_fanin(node, [&]( auto si ) {
      fanin.push_back(xag.node_to_index(xag.get_node(si)));
    } );
    fi[xag.node_to_index( node )] = sym_diff( fi[fanin[0]], fi[fanin[1]] );
    assert(!fi[xag.node_to_index( node )].empty());
  }
}


static inline  std::vector<std::vector<uint32_t>> get_fi (xag_network const& xag, std::vector<node_t> const& drivers )
{
  std::vector<std::vector<uint32_t>> fi (xag.size());
  xag.foreach_node( [&]( auto n ) {
    update_fi(n, xag, fi, drivers);
  });
  return fi;
}

inline  std::vector<cone_t> get_cones( node_t node, xag_network const& xag, std::vector<std::vector<uint32_t>> const& fi, bool include_root = true )
{
  std::vector<cone_t> cones; 

  xag.foreach_fanin( node, [&]( auto si ) {
    auto fanin = xag.get_node( si );

    auto set = cone_t(fanin, fi[xag.node_to_index( fanin )], xag.is_complemented(si) ) ; 
    cones.push_back( set ); 
  } );
  assert( cones.size() == 2 );

  // if one fanin is AND or PI or PO symply delete f
  if ( (cones[0].leaves.size() == 1) != (cones[1].leaves.size() == 1) )
  {
    if ( (cones[0].leaves.size() == 1) )
      std::reverse(cones.begin(), cones.end());

    cones[0].target = cones[0].leaves;

    /* remove the single leaf if there is overlap */
    /* and swap */
    auto it = std::find(cones[0].target.begin(), cones[0].target.end(), cones[1].leaves[0]);
    if (it != cones[0].target.end())
    {
      cones[0].target.erase( it );
      std::reverse(cones.begin(), cones.end());
    }

  }
  else 
  {
    if ( is_included(fi[xag.node_to_index(cones[1].root)], fi[xag.node_to_index(cones[0].root)]) )
    { 
      std::reverse(cones.begin(), cones.end());
    }

    auto left = xag.node_to_index(cones[0].root);
    auto right = xag.node_to_index(cones[1].root);


    /* search a target for first */
    /* empty if left is included TODO: remove this */
    std::set_difference(fi[left].begin(), fi[left].end(), 
      fi[right].begin(), fi[right].end(), std::inserter(cones[0].target, cones[0].target.begin()));

    /* set difference */
    std::set_difference(fi[right].begin(), fi[right].end(), 
      fi[left].begin(), fi[left].end(), std::inserter(cones[1].target, cones[1].target.begin()));
    
    /* the first may be included */
    if( include_root && is_included(fi[left], fi[right] ) )
    {
      /* add the top of the cone to the right */
      cones[1].target.push_back( cones[0].root ); 
      cones[1].leaves = cones[1].target;

      /* anything can be chosen as target */ 
      cones[0].target = fi[left];
    }
  }

  return cones;
}

static inline steps_xag_t gen_steps( node_t node, std::vector<cone_t> cones, bool compute)
{
  steps_xag_t comp_steps;
  
  for(auto ch : cones)
  {
    assert(ch.copies.empty());
    if(ch.is_buffer())
    {
      comp_steps.push_back({ch.root, buffer_action{ch.root, ch.leaves[0]}});
    }
    else if(!ch.is_and_or_pi())
    {
      if ( !ch.target.empty() )
      {
        comp_steps.push_back( {ch.root, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves}} );
      }
      else 
      {
        comp_steps.push_back( {ch.root, compute_action{ ch.leaves, std::nullopt}} );
      }
    }

  }
  
  if(compute)
    comp_steps.push_back( {node, compute_action{}} );
  else
    comp_steps.push_back( {node, uncompute_action{}} );


  std::reverse( cones.begin(), cones.end() );

  for(auto ch : cones)
  {
    if(ch.is_buffer())
    {
      comp_steps.push_back({ch.root, buffer_action{ch.leaves[0], ch.root}});
    }
    else if(!ch.is_and_or_pi())
    {
      if ( !ch.target.empty() )
      {
        comp_steps.push_back( {ch.root, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves}} );
      }
      else 
      {
        comp_steps.push_back( {ch.root, compute_action{ ch.leaves, std::nullopt}} );
      }
    }
   
  }
  
  return comp_steps;
}


static inline std::vector<std::vector<node_t>> get_levels( xag_network const& xag_t, std::vector<node_t> const& drivers)
{
  mockturtle::depth_view<xag_network, xag_depth_cost<xag_network>> xag {xag_t};

  std::vector<std::vector<node_t>>levels (xag.depth());
                                                                 
  xag.foreach_gate( [&]( auto n ) {
    if( xag.is_and(n) || std::find( drivers.begin(), drivers.end(), n ) != drivers.end() ) 
      levels[xag.level(n)-1].push_back(n);
  });

  return levels;
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
class xag_mapping_strategy : public mapping_strategy<xag_network>
{

public:
  bool compute_steps( xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);                                                     
    auto fi  = get_fi(xag, drivers);
    auto it = steps().begin();

    xag.foreach_gate( [&]( auto node ) {
      
      if ( xag.is_and( node ) || std::find( drivers.begin(), drivers.end(), node ) != drivers.end() )
      {
        auto cones = get_cones(  node, xag, fi );

        if ( xag.is_and( node ))
        {
          /* compute step */
          auto cc = gen_steps( node , cones,  true);
          it = steps().insert( it, cc.begin(), cc.end() );
          it = it + cc.size();

          if ( std::find( drivers.begin(), drivers.end(), node ) == drivers.end()  )
          { 
            auto uc = gen_steps( node , cones, false);
            it = steps().insert( it, uc.begin(), uc.end() );
          }
        }
        /* node is an XOR output */
        else 
        {
          auto xc = gen_steps( node, cones, true);
          it = steps().insert( it, xc.begin(), xc.end() );
          it = it + xc.size();
        }
      }

    } );

    return true;
  }
};

class xag_fast_lowt_mapping_strategy : public mapping_strategy<xag_network>
{

public:
  bool compute_steps( xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);                                                     
    auto fi  = get_fi(xag, drivers);
    auto levels = get_levels(xag, drivers);
    auto it = steps().begin();

    for(auto lvl : levels)
    { 
      /* store two action sets for each node in the level */
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> node_and_action;      
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> to_be_uncomputed;

      for(auto n : lvl)
      {
        /* this strategy does not support symplification of an included fanin cone, hence the false flag */
        auto cones = get_cones(n, xag, fi, false);
        node_and_action.push_back({n, cones});
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

#ifdef USE_Z3
/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It creates an abstract graph from the XAG, each box node in the abstract graph corresponds to 
    AND nodes and its linear transitive fanin cones.
    Pebbling is played on the abstract graph.
  \endverbatim
*/
class xag_pebbling_mapping_strategy : public mapping_strategy<xag_network>
{
 
  steps_xag_t get_box_steps(steps_abs_t const& store_steps)
  {
    steps_xag_t xag_steps;
    for(auto step : store_steps)
    {
      auto box_node = step.first;
      auto action = step.second;

      auto [node, cones] = box_to_action[box_node];

      auto cc = gen_steps( node, cones, std::holds_alternative<compute_action>( action ));
      xag_steps.insert(xag_steps.end(), cc.begin(), cc.end());
    }

    return xag_steps;
  }

  abstract_network build_box_network(xag_network const& xag, bool use_w = false)
  {
    abstract_network box_ntk;

    std::unordered_map<node_t, abstract_network::signal> xag_to_box;
    

    xag.foreach_pi([&] (auto pi)
    {
      xag_to_box[pi] = box_ntk.create_pi(); 
    });

    auto drivers = detail::get_outputs(xag);
    auto fi = get_fi (xag, drivers);

    xag.foreach_gate([&] (auto node)
    {
      if(xag.is_and(node) || std::find(drivers.begin(), drivers.end(), node) != drivers.end())
      {
        /* collect all the input signals of the box */
        std::vector<abstract_network::signal> box_ins;
        boost::dynamic_bitset<> visited (xag.size());

        auto cones = get_cones(  node, xag, fi );
        auto tot_leaves = cones[0].leaves.size() + cones[1].leaves.size();
        for( auto c : cones)
        {
          for (auto l : c.leaves)
          {
            if(!visited[l])
              box_ins.push_back(xag_to_box[l]);
            visited.set(l);
          }
        }
        
        
        auto box_node_s = use_w ? box_ntk.create_node(box_ins, (int)tot_leaves) : box_ntk.create_node(box_ins);
        auto box_node = box_ntk.get_node(box_node_s);

        xag_to_box[node] = box_node_s;

        box_to_action[box_node] = {node, cones};
       
        
        if(std::find(drivers.begin(), drivers.end(), node) != drivers.end())
        {
          box_ntk.create_po(box_node_s);
        }
      }
    });

    return box_ntk;
  }

  pebbling_mapping_strategy_params ps;
  std::unordered_map<abstract_network::node, std::pair<node_t, std::vector<cone_t>> > box_to_action;

public: 
  
  xag_pebbling_mapping_strategy( pebbling_mapping_strategy_params const& ps = {} )
  : ps( ps ) {}

  bool compute_steps( xag_network const& ntk ) override
  {
    using solver_t = z3_pebble_solver<abstract_network>;
    mockturtle::topo_view xag {ntk};
    steps_abs_t store_steps;
    

    if(ps.progress) std::cout << "[i]  Generate box network... \n";
    auto box_ntk = build_box_network(xag, ps.optimize_weight);

    store_steps = pebble<solver_t, abstract_network> (box_ntk, ps);

    if ( store_steps.empty() )
      return false;


    if(ps.progress) std::cout << "[i] Extract the XAG strategy ...\n";

    this -> steps() = get_box_steps(store_steps);
    return true;
  }
};
#endif



/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It finds a strategy that aim to minimize the number of T-stages or T-depth.
    Every level uses some extra-qubits to copy AND inputs.
  \endverbatim
*/
class xag_low_depth_mapping_strategy : public mapping_strategy<xag_network>
{
  void eval_copies(std::vector<cone_t>& cones, boost::dynamic_bitset<>& visited)
  {
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


public: 
  
  bool compute_steps( xag_network const& ntk ) override
  {
    // the strategy proceeds in topological order and level by level
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);

    /* iterate the xag to extract transitive fanin cones for all nodes */
    auto fi = get_fi(xag, drivers);

    /* each m_level is filled with AND nodes and XOR outputs */
    auto levels = get_levels(xag, drivers);
    auto it = steps().begin();

    for(auto lvl : levels){ if(lvl.size() != 0)
    {

      /* store two action sets for each node in the level */
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> node_and_action;      
      std::vector<std::pair<uint32_t, std::vector<cone_t>>> to_be_uncomputed;


      boost::dynamic_bitset<> visited (xag.size());
      for(auto n : lvl)
      {
        /* this strategy does not support symplification of an included fanin cone, hence the false flag */
        auto cones = get_cones(n, xag, fi, false);
        if(xag.is_and(n))
        {
          /* modifies cones.leaves and cones.copies according to visited */
          eval_copies(cones, visited);
        }
        node_and_action.push_back({n, cones});
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

class xag_depth_fit_mapping_strategy : public mapping_strategy<xag_network>
{

  std::vector<boost::dynamic_bitset<>> get_mask( 
  std::vector<uint64_t> const& lvl, 
  xag_network const& xag, 
  std::vector<std::vector<uint32_t>> const& fi)
  {
    std::vector<boost::dynamic_bitset<>> masks (lvl.size());
    for(uint32_t i = 0; i < lvl.size(); i++)
    {  
      masks[i].resize(xag.size());
      auto cones = get_cones(lvl[i], xag, fi, false);  
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

  void build_compatibility_graph(igraph_t* g, std::vector<uint64_t> const& lvl, xag_network const& xag, std::vector<std::vector<uint32_t>> const& fi)
  {
    auto conflicts = get_mask(lvl, xag, fi);

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
  uint32_t qubits_offset = 0;
  uint32_t size_limit= std::numeric_limits<uint>::max();

public: 

  xag_depth_fit_mapping_strategy(uint32_t max_size_comp_graph = std::numeric_limits<uint>::max())
  : size_limit(max_size_comp_graph) {}

  uint32_t get_offset() { return qubits_offset;}

  bool compute_steps( xag_network const& ntk ) override
  {
    /* set up graph attribute handler */
    igraph_i_set_attribute_table(&igraph_cattribute_table);

    // the strategy proceeds in topological order
    mockturtle::topo_view xag {ntk};

    auto drivers = detail::get_outputs(xag);

    auto fi = get_fi (xag, drivers);
    
    /* each m_level is filled with AND nodes and XOR outputs */
    /* fi is propagated */
    auto levels = get_levels(xag, drivers);                              

    auto it = steps().begin();

    for(auto lvl : levels)
    {
      assert(lvl.size()!=0);

      igraph_t graph; 
      igraph_vector_ptr_t subgraphs; 
      igraph_vector_ptr_init(&subgraphs, 0);

      build_compatibility_graph(&graph, lvl, xag, fi);
      
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
            to_be_computed.push_back({n, get_cones(n, xag, fi, false)});
          }

          it = steps().insert(it, {clique[0], compute_level_action{to_be_computed}});
          it = it + 1;
        
          for(auto& n : clique)
          {
            if(std::find(drivers.begin(), drivers.end(), n) == drivers.end())
              to_be_uncomputed.push_back({n, get_cones(n, xag, fi, false)});
          }

          it = steps().insert(it, {clique[0], uncompute_level_action{to_be_uncomputed}});

          if(clique.size()>qubits_offset)
            qubits_offset = clique.size();
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