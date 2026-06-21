// Copyright (c) 2026 Capgemini Engineering Research and Development.
//
// This file is part of OCCT-Light software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Affero General Public License version 3 as published
// by the Free Software Foundation, with an option to use any later version.
// Consult the file LICENSE_AGPL_30.txt included in OCCT-Light distribution
// for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of a commercial
// license or contractual agreement.
//
// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef OCCTL_TOPO_DERIVED_STATE_OVERRIDES_HXX
#define OCCTL_TOPO_DERIVED_STATE_OVERRIDES_HXX

#include <BRepGraph.hxx>
#include <map>

namespace OcctL::Topo::DerivedState
{

// Thread-local override maps for derived state flags with no public OCCT setter.
// Keyed by BRepGraph pointer -> edge index (int) -> flag value (bool).
// Implementation is in DerivedStateOverrides.cxx to ensure single instances across TUs.

const std::map<int, bool>* GetSameParamOverrides(const BRepGraph* theGraph);
void SetSameParamOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue);
const std::map<int, bool>* GetSameRangeOverrides(const BRepGraph* theGraph);
void SetSameRangeOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue);
const std::map<int, bool>* GetEdgeClosedOverrides(const BRepGraph* theGraph);
void SetEdgeClosedOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue);
const std::map<int, bool>* GetDegenerateOverrides(const BRepGraph* theGraph);
void SetDegenerateOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue);

// Thread-local override for wire closure (keyed by wire index).
const std::map<int, bool>* GetWireClosedOverrides(const BRepGraph* theGraph);
void SetWireClosedOverride(BRepGraph* theGraph, int theWireIndex, bool theValue);

} // namespace OcctL::Topo::DerivedState

#endif // OCCTL_TOPO_DERIVED_STATE_OVERRIDES_HXX
