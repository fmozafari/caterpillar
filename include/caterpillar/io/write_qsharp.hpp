#include <fstream>
#include <iostream>
#include <fmt/format.h>
#include <tweedledum/networks/netlist.hpp>
#include <caterpillar/structures/stg_gate.hpp>


namespace caterpillar
{
void write_qsharp( tweedledum::netlist<stg_gate> const& rnet, std::ostream& os, std::string const& funcname, bool meas_based_uncomp = false)
{
  auto qubits = rnet.num_qubits();

  //check repeting ANDs for uncomputation
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> ash;
  os << fmt::format("operation {}(): Unit\n", funcname ) ;
  os << "{\n";
  os << fmt::format("  using (qs = Qubit[{}])\n", qubits);
  os << "  {\n";


  rnet.foreach_cgate([&] (const auto rgate)
  {
    auto cs = rgate.gate.controls();
    auto ts = rgate.gate.targets();
    
    for( auto c : cs)
    {
      if(c.is_complemented())
        os << fmt::format("    X(qs[{}]);\n", c);;
    }

    if(rgate.gate.num_controls() == 0)
    {
      //NOT
      os << fmt::format("    X(qs[{}]);\n", ts[0]);

    }
    else if (rgate.gate.num_controls() == 1)
    {
      //CNOT
      os << fmt::format("    CNOT(qs[{}], qs[{}] );\n", cs[0], ts[0]);

    }
    else if (rgate.gate.num_controls() == 2)
    {
      //AND
      auto sign = std::make_tuple(cs[0], cs[1], ts[0]);
      if (std::find(ash.begin(), ash.end(), sign ) == ash.end())
      {
        
        meas_based_uncomp ? 
          os << fmt::format("    AND(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]) : 
          os << fmt::format("    CCNOT(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]);

        ash.push_back(sign);
      }
      else
      {
        meas_based_uncomp ? 
          os << fmt::format("    Adjoint AND(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]) : 
          os << fmt::format("    CCNOT(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]);
      }

    }
    else if (rgate.gate.num_controls() > 2)
    {
      //multiple-controlled X
      os << fmt::format("    Controlled XWrap ([ qs[{}] ], qs[{}]);\n", fmt::join(cs, "], qs["), ts[0]);

    }
    for( auto c : cs)
    {
      if(c.is_complemented())
        os << fmt::format("    X(qs[{}]);\n", c);;
    }
    

  });

  os << "\n  }\n}\n";

}



void write_qsharp( tweedledum::netlist<stg_gate> const& rnet, std::string const& filename, std::string const& funcname = "", bool meas_based = false )
{
  std::ofstream os( filename.c_str(), std::ofstream::app );
  write_qsharp( rnet, os, funcname, meas_based );
  os.close();
}

}//namespace caterpillar