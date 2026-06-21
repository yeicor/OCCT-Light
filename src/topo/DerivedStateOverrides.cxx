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

#include "DerivedStateOverrides.hxx"

#include <mutex>

namespace OcctL::Topo::DerivedState
{
namespace
{
  std::mutex s_mutex;
  std::map<const BRepGraph*, std::map<int, bool>> s_sameParamOverrides;
  std::map<const BRepGraph*, std::map<int, bool>> s_sameRangeOverrides;
  std::map<const BRepGraph*, std::map<int, bool>> s_edgeClosedOverrides;
  std::map<const BRepGraph*, std::map<int, bool>> s_degenerateOverrides;
  std::map<const BRepGraph*, std::map<int, bool>> s_wireClosedOverrides;
} // namespace

const std::map<int, bool>* GetSameParamOverrides(const BRepGraph* theGraph)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  const auto aIt = s_sameParamOverrides.find(theGraph);
  return aIt != s_sameParamOverrides.end() ? &aIt->second : nullptr;
}

void SetSameParamOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  s_sameParamOverrides[theGraph][theEdgeIndex] = theValue;
}

const std::map<int, bool>* GetSameRangeOverrides(const BRepGraph* theGraph)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  const auto aIt = s_sameRangeOverrides.find(theGraph);
  return aIt != s_sameRangeOverrides.end() ? &aIt->second : nullptr;
}

void SetSameRangeOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  s_sameRangeOverrides[theGraph][theEdgeIndex] = theValue;
}

const std::map<int, bool>* GetEdgeClosedOverrides(const BRepGraph* theGraph)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  const auto aIt = s_edgeClosedOverrides.find(theGraph);
  return aIt != s_edgeClosedOverrides.end() ? &aIt->second : nullptr;
}

void SetEdgeClosedOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  s_edgeClosedOverrides[theGraph][theEdgeIndex] = theValue;
}

const std::map<int, bool>* GetDegenerateOverrides(const BRepGraph* theGraph)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  const auto aIt = s_degenerateOverrides.find(theGraph);
  return aIt != s_degenerateOverrides.end() ? &aIt->second : nullptr;
}

void SetDegenerateOverride(BRepGraph* theGraph, int theEdgeIndex, bool theValue)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  s_degenerateOverrides[theGraph][theEdgeIndex] = theValue;
}

const std::map<int, bool>* GetWireClosedOverrides(const BRepGraph* theGraph)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  const auto aIt = s_wireClosedOverrides.find(theGraph);
  return aIt != s_wireClosedOverrides.end() ? &aIt->second : nullptr;
}

void SetWireClosedOverride(BRepGraph* theGraph, int theWireIndex, bool theValue)
{
  std::lock_guard<std::mutex> aLock(s_mutex);
  s_wireClosedOverrides[theGraph][theWireIndex] = theValue;
}

} // namespace OcctL::Topo::DerivedState
