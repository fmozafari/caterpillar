/*------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*-----------------------------------------------------------------------------*/
#pragma once

#ifdef USE_Z3 
#include <caterpillar/structures/pebbling_view.hpp>
#include <caterpillar/structures/abstract_network.hpp>
#include <caterpillar/synthesis/strategies/action.hpp>
#include <fmt/format.h>

#include <z3++.h>
#include <vector>
#include <type_traits>

#include <mockturtle/networks/klut.hpp>


namespace caterpillar
{
using namespace z3;
using namespace mockturtle;

template<typename Ntk>
class z3_pebble_inplace_solver
{
  
  struct variables
  {
    variables( context& ctx )
        : s( expr_vector( ctx ) ), a( expr_vector( ctx ) ), i(expr_vector(ctx)) {}

    expr_vector s;
    expr_vector a;
		expr_vector i;
	};

public:

	using node = node<Ntk>;
  using result = z3::check_result;

	z3_pebble_inplace_solver(const Ntk& net, const int& pebbles, const int& max_conflicts = 0, const int& max_weight = 0)
	:_net(net), _pebbles(pebbles+_net.num_pis()+1), _max_weight(max_weight), slv(solver(ctx)), curr(variables(ctx)), next(variables(ctx))
	{
		static_assert( has_get_node_v<Ntk>, "Ntk does not implement the get_node method" );
		static_assert( has_foreach_po_v<Ntk>, "Ntk does not implement the foreach_po method" );
		static_assert( has_foreach_node_v<Ntk>, "Ntk does not implement the foreach_node method" );
		static_assert( has_foreach_fanin_v<Ntk>, "Ntk does not implement the foreach_fanin method" );
		static_assert( has_get_weight_v<Ntk>, "Ntk does not implement the get_weight method" );
		static_assert( has_get_parents_v<Ntk>, "Ntk does not implement the get_parents method" );

		static_assert( !(std::is_same_v<Ntk, klut_network>) );

		ctx.set( "max_conflicts", max_conflicts );
	}

	uint32_t node_to_var( node n )
	{
		return n ;
	}

	node var_to_node(uint32_t var)
	{
		return var;
	}

	expr_vector new_variable_set(std::string s)
	{
		expr_vector x (ctx);

		_net.foreach_node([&](auto node){
			auto x_name = fmt::format("{}_{}_{}", s, num_steps, node_to_var(node));
			x.push_back( ctx.bool_const( x_name.c_str() ) );
			
		});

		return x;
	}

	/* for inplace I need input pebbles */
	void init()
	{
		expr_vector invar (ctx);
		expr_vector nodevar (ctx);	

		curr.s = new_variable_set("s");
		curr.a = new_variable_set("a");
		curr.i = new_variable_set("i");

		for( auto i = 0u; i < _net.num_pis()+1; i++)
		{
			invar.push_back(curr.s[i]);
		}
		for( auto i = _net.num_pis()+1; i < curr.s.size() ; i++)
		{
			nodevar.push_back(curr.s[i]);
		}

		slv.add(!mk_or(nodevar));
		slv.add(mk_and(invar));
		slv.add(!mk_or(curr.a));
		slv.add(!mk_or(curr.i));

		//std::cout << "Init: \n";
		//std::cout << "invar: " << invar << " nodevars: " <<  nodevar << " actionsvar: " << curr.a << "\n";
	}

	uint32_t current_step()
	{
		return num_steps;
	}

	result unsat() { return result::unsat; }
	result sat() { return result::sat; }
	result unknown() { return result::unknown; }


	void add_step()
	{
		num_steps+=1;

		next.s = new_variable_set("s");
		next.a = new_variable_set("a");
		next.i = new_variable_set("i");

		for (auto var=0u ; var<next.s.size(); var++)
		{
			/* pis clauses */
			if(var < _net.num_pis()+1)
			{
				slv.add(!next.a[var]);
				slv.add( implies( curr.s[var] != next.s[var], next.i[var]) );
			}

			/* inplace encoding, should support also out of place!! */
			if (_net.is_xor(var))
			{
				std::vector<uint32_t> chs;
				_net.foreach_fanin( var, [&] ( auto sig ) {
					chs.push_back(_net.get_node( sig ));
				});

				assert (chs.size() == 2);
				auto comp = next.a[var] && next.s[var];
				auto uncomp = next.a[var] && !next.s[var];
				auto next_input_different =  next.s[chs[0]] != next.s[chs[1]] ;
				auto curr_input_different = curr.s[chs[0]] != curr.s[chs[1]]  ;
				auto input_pebbled_in_step =  next.s[chs[0]] && 
																			curr.s[chs[0]] && 
																			next.s[chs[1]] &&
																			curr.s[chs[1]];

				 
				slv.add ( implies (comp, 
													( 
														(curr.s[chs[0]] && curr.s[chs[1]] && next_input_different) ||
														input_pebbled_in_step
													) ));
				slv.add ( implies (uncomp, 
													(	
														(curr_input_different && next.s[chs[0]] && next.s[chs[1]]) ||
														input_pebbled_in_step
													) ));

				slv.add ( implies ( next.a[var], !next.a[chs[0]] && !next.a[chs[1]] ) );

				slv.add ( implies ( next.i[chs[0]], next.a[var] && (curr.s[chs[0]] != next.s[chs[0]]) ) );
				slv.add ( implies ( next.i[chs[1]], next.a[var] && (curr.s[chs[1]] != next.s[chs[1]]) ) );

				slv.add( implies ( curr.s[var] == next.s[var], !next.a[var] && !next.i[var] ) );
				slv.add( implies ( curr.s[var] != next.s[var], next.a[var] || next.i[var] ) );
			}
			/* out of place encoding */
			else if (var > _net.num_pis())
			{
				_net.foreach_fanin( var_to_node( var ), [&]( auto sig ) {

					auto ch = _net.get_node( sig );
					slv.add( implies (next.a[var], ( curr.s[ch] && next.s[ch] && !next.a[ch] ) ) );
				
				} );
				slv.add( implies ( curr.s[var] != next.s[var] , next.a[var] ) );
				slv.add( implies ( curr.s[var] == next.s[var] , !next.a[var] ) );
			}
			
			//block condition on inputs whose parents don't change
			/*block condition on nodes with multiple fanouts. */
			if( !_net.get_parents(var).empty() )
			{
				expr_vector parents_vars(ctx);
				for( auto parent : _net.get_parents(var))
				{
					if (_net.is_xor(parent))
						parents_vars.push_back(next.a[parent]);
				}
				if( !parents_vars.empty())
				{
					slv.add( implies( next.i[var], atleast(parents_vars, 1) && atmost(parents_vars, 1)) );
				}
				else
				{
					slv.add( !next.i[var] );
				}
			}
			else 
			{
				slv.add( !next.i[var] );
			}
				
		}

		if(_pebbles != 0)	slv.add(atmost(next.s, _pebbles));
		curr = next;
	}

	/* I can only consider a for the weights, as for every i there is also an a and it 
	 * only corresponds to one operation, namely a CNOT 
	 * */
	expr_vector weight_expr()
	{
		expr_vector clause (ctx);
		for (uint32_t k=0; k<num_steps+1; k++)
		{
			for (uint32_t i=0; i<curr.s.size(); i++)
			{
				for (uint32_t r=0; r<_net.get_weight(var_to_node(i)); r++)
				{
					clause.push_back(ctx.bool_const(fmt::format("a_{}_{}", k, i).c_str()));
				}
			}
		}
		return clause;
	}

	result solve()
	{
		std::vector<uint32_t> o_nodes;

		slv.push();
		_net.foreach_po([&](auto po_sign)
		{
			o_nodes.push_back(_net.get_node(po_sign));
		});

		/* add final clauses */
		for (auto var=0u ; var<next.s.size(); var++)
		{
			if(std::find(o_nodes.begin(), o_nodes.end(), var_to_node(var)) != o_nodes.end())
			{
				slv.add(curr.s[var]);
			}
			else if ( var < _net.num_pis() + 1)
			{
				slv.add(curr.s[var]);
			}
			else
			{
				slv.add(!curr.s[var]);
			}
			
		}

		/* add weight clause */
		if constexpr ( has_get_weight_v<Ntk> )
		{
			if(_max_weight != 0) slv.add(atmost(weight_expr(), _max_weight));
		}

		/* check result (drop final clauses if unsat)*/
		auto result = slv.check();
		if (result == unsat())
		{
			slv.pop();
		}

		return result;
	}


	void print()
	{

		model m = slv.get_model();
		uint32_t w = 0;

		for(uint32_t n=0; n<curr.s.size(); n++)
		{
			std::cout << std::endl;
			for(uint32_t k =0; k<num_steps+1; k++)
			{
				auto s = fmt::format("s_{}_{}", k, n);
				auto s_var = m.eval(ctx.bool_const(s.c_str()));
				if (s_var.is_true()) std::cout << "1" << "-";
				else std::cout << "0" << "-";

				auto a = fmt::format("a_{}_{}", k, n);
				auto a_var = m.eval(ctx.bool_const(a.c_str()));
				if (_max_weight !=0)
				{
					if (a_var.is_true()) 
					{
						w += _net.get_weight(var_to_node(n));
						std::cout << "y" << "+" << _net.get_weight(var_to_node(n)) << " " ;
					}
					else std::cout << "n" << "+0 ";
				}
				
			}
		}
		std::cout << fmt::format("\nTOT.Weight = {}\n", w);

		std::cout << "a var\n";
		for(uint32_t n=0; n<curr.s.size(); n++)
		{
			for(uint32_t k =0; k<num_steps+1; k++)
			{
				auto a = fmt::format("a_{}_{}", k, n);
				auto a_var = m.eval(ctx.bool_const(a.c_str()));
				if (a_var.is_true()) std::cout << "1" << "-";
				else std::cout << "0" << "-";
			}
			std::cout << std::endl;

		}

		std::cout << "i var\n";
		for(uint32_t n=0; n<curr.s.size(); n++)
		{
			std::cout << n << " ";
			for(uint32_t k =0; k<num_steps+1; k++)
			{
				auto i = fmt::format("i_{}_{}", k, n);
				auto i_var = m.eval(ctx.bool_const(i.c_str()));
				if (i_var.is_true()) std::cout << "1" << "-";
				else std::cout << "0" << "-";
			}
			std::cout << std::endl;
		}

	}

	std::vector<std::pair<mockturtle::node<pebbling_view<Ntk>>, mapping_strategy_action>> extract_result( bool verbose = false)
	{
		model m = slv.get_model();
		std::vector<std::pair<mockturtle::node<pebbling_view<Ntk>>, mapping_strategy_action>> steps;

		for (uint32_t k = 0; k <num_steps+1; k++)
		{
			/* pair<node, ? computing : uncomputing> */
			std::vector<std::pair<uint32_t, bool>> step_action;

			for (uint32_t i = 0; i< curr.s.size(); i++)
			{
				auto a_var = fmt::format("a_{}_{}", k, i).c_str();
				if( m.eval(ctx.bool_const(a_var)).is_true())
				{
					bool s_pre = m.eval( ctx.bool_const( fmt::format("s_{}_{}", k-1, i).c_str() )).is_true();
					bool s_cur = m.eval( ctx.bool_const( fmt::format("s_{}_{}", k, i).c_str() )).is_true();
					assert (s_pre != s_cur);

					step_action.push_back(std::make_pair(i, s_cur));
				}
			}

			/* sort step actions to have all the deactivations (false) first */
			std::sort(step_action.begin(), step_action.end(), 
				[](const std::pair<uint32_t, bool>& first, const std::pair<uint32_t, bool>& second)
				{
					(void)second;

					if(!first.second)	return true;
					else return false;
				}
			);

			/* add actions to the pebbling strategy */
			for(auto act : step_action)
			{
				auto act_node = var_to_node(act.first);
				uint64_t act_ch_node;
				bool inplace = false;

				_net.foreach_fanin(act_node, [&] (const auto fi)
				{
					uint64_t node_fi = _net.get_node(fi);
					auto i_var = fmt::format("i_{}_{}", k, node_fi).c_str();
					if (m.eval(ctx.bool_const(i_var)).is_true())
					{
						inplace = true;
						act_ch_node = node_fi;
					}
				});

				if(inplace)
				{
					auto target = static_cast<uint32_t>(act_ch_node);
					if (act.second)
					{
						steps.push_back({act_node, compute_inplace_action{target, {}}});
						if( verbose ) std::cout << "compute node " <<  act_node << " inplace on " << target << std::endl;
					}
					else
					{
						steps.push_back({act_node, uncompute_inplace_action{target, {}}});
						if( verbose ) std::cout << "uncompute node " <<  act_node << " inplace on " << target << std::endl;
					}
				}
				else if(act.second)
				{
					steps.push_back({act_node, compute_action{}});
					if( verbose ) std::cout << "compute node " <<  act_node << std::endl;
				}
				else
				{ 
					steps.push_back({act_node, uncompute_action{}});
					if( verbose ) std::cout << "uncompute node " <<  act_node << std::endl;
				}
				
				
			}
		}


		return steps;
	}


private:
const Ntk _net;
const int _pebbles;
const int _max_weight;

context ctx;
solver slv;

uint32_t num_steps = 0;
variables curr;
variables next;

};


}
#endif