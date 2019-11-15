#include <fstream>
#include <iostream>
#include <fmt/format.h>
#include <tweedledum/networks/netlist.hpp>
#include <caterpillar/structures/stg_gate.hpp>


namespace caterpillar
{
void write_qsharp( tweedledum::netlist<stg_gate> const& rnet, std::ostream& os, std::string const& funcname, bool low_depth = false)
{
  auto qubits = rnet.num_qubits();

  //check repeting ANDs for uncomputation
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> ash;
  os << fmt::format("operation Function{}(): Unit\n", funcname ) ;
  os << "{\n";
  os << fmt::format("  using (qs = Qubit[{}])\n", qubits);
  os << "  {\n";


  rnet.foreach_cgate([&] (const auto rgate)
  {
    auto cs = rgate.gate.controls();
    auto ts = rgate.gate.targets();

    if(rgate.gate.num_controls() == 0)
    {
      //NOT
      os << fmt::format("    X(qs[{}]);\n", ts[0]);

    }
    if (rgate.gate.num_controls() == 1)
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
        
        low_depth ? os << fmt::format("    ANDLowDepth(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]) : os << fmt::format("    AND(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]);

        ash.push_back(sign);
      }
      else
      {
        low_depth ? os << fmt::format("    Adjoint ANDLowDepth(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]) : os << fmt::format("    Adjoint AND(qs[{}], qs[{}], qs[{}]);\n", cs[0], cs[1], ts[0]);
      }

    }
    else if (rgate.gate.num_controls() > 2)
    {
      //multiple-controlled X
      os << fmt::format("    Controlled X ([ qs[{}] ], qs[{}]);\n", fmt::join(cs, "], qs["), ts[0]);

    }
    

  });

  os << "\n  }\n}\n";

}



void write_qsharp( tweedledum::netlist<stg_gate> const& rnet, std::string const& filename, std::string const& funcname = "" )
{
  std::ofstream os( filename.c_str(), std::ofstream::out );
  write_qsharp( rnet, os, funcname );
  os.close();
}

}//namespace caterpillar