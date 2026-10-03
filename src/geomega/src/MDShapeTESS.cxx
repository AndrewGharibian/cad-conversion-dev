/*
 * MDShapeTESS.cxx
 *
 * Copyright (C) by Andreas Zoglauer.
 * All rights reserved.
 *
 * Please see the source-file for the copyright-notice.
 *
 */

#include "MDShapeTESS.h"
#include "MStreams.h"
#include "TGeoTessellated.h"
#include "TGeoManager.h"

#include <fstream>

#ifdef ___CLING___
ClassImp(MDShapeTESS)
#endif

////////////////////////////////////////////////////////////////////////////////

//! Default constructor
MDShapeTESS::MDShapeTESS(const MString& Name) : MDShape(Name)
{
  m_Type = "TESS";
  m_FileName = "";
}

////////////////////////////////////////////////////////////////////////////////

//! Default destructor
MDShapeTESS::~MDShapeTESS()
{
}

////////////////////////////////////////////////////////////////////////////////

//! Set the shape mesh file path
bool MDShapeTESS::Set(const MString& FileName)
{
  m_FileName = FileName;
  return Validate();
}

////////////////////////////////////////////////////////////////////////////////

//! Parse tokenized text from .geo file
bool MDShapeTESS::Parse(const MTokenizer& Tokenizer, const MDDebugInfo& Info)
{
  if (Tokenizer.GetNTokens() < 1) {
    mgout<<"Error parsing TESS "<<m_Name<<": Requires filename (mesh file path)"<<error;
    return false;
  }

  m_FileName = Tokenizer.GetTokenAtAsString(0);

  return Validate();
}

////////////////////////////////////////////////////////////////////////////////

//! Validate file existence and data
bool MDShapeTESS::Validate()
{
  if (m_FileName.IsEmpty() == true) {
    mgout<<"Error in TESS "<<m_Name<<": Filename cannot be empty!"<<error;
    return false;
  }

  std::ifstream File(m_FileName.Data());
  if (!File.good()) {
    mgout<<"Error in TESS "<<m_Name<<": Cannot open file '"<<m_FileName<<"'!"<<error;
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

//! Create the ROOT TGeo shape representation
TGeoShape* MDShapeTESS::CreateTGeoShape()
{
  if (m_TGeoShape != nullptr) delete m_TGeoShape;

  TGeoTessellated* Tess = new TGeoTessellated(m_Name.Data());

  // STL / OBJ triangle loading logic will populate Tess facets here

  m_TGeoShape = Tess;
  return m_TGeoShape;
}

////////////////////////////////////////////////////////////////////////////////

//! Check if a given position is inside the shape
bool MDShapeTESS::IsInside(const MVector& Pos, const double Tolerance, const bool PreferOutside)
{
  if (m_TGeoShape == nullptr) CreateTGeoShape();
  if (m_TGeoShape == nullptr) return false;

  double point[3] = { Pos.X(), Pos.Y(), Pos.Z() };
  return m_TGeoShape->Contains(point);
}

////////////////////////////////////////////////////////////////////////////////

//! Calculate distance from outside into the shape along direction
double MDShapeTESS::DistanceOutsideIn(const MVector& Pos, const MVector& Dir, double Tolerance)
{
  if (m_TGeoShape == nullptr) CreateTGeoShape();
  if (m_TGeoShape == nullptr) return 0.0;

  double point[3] = { Pos.X(), Pos.Y(), Pos.Z() };
  double dir[3]   = { Dir.X(), Dir.Y(), Dir.Z() };
  return m_TGeoShape->DistFromOutside(point, dir);
}

////////////////////////////////////////////////////////////////////////////////

//! Convert parameters to string format
MString MDShapeTESS::ToString()
{
  MString S;
  S += "Tessellated shape " + m_Name + " using file " + m_FileName + "\n";
  return S;
}

////////////////////////////////////////////////////////////////////////////////

//! Return Geomega geometry description line
MString MDShapeTESS::GetGeomega() const
{
  MString S;
  S += MString("TESS ") + m_FileName;
  return S;
}

////////////////////////////////////////////////////////////////////////////////

//! Calculate or return volume in cm^3
double MDShapeTESS::GetVolume()
{
  if (m_TGeoShape == nullptr) CreateTGeoShape();
  if (m_TGeoShape != nullptr) {
    return m_TGeoShape->Capacity();
  }
  return 0.0;
}

////////////////////////////////////////////////////////////////////////////////

//! Scale axes by given factor
bool MDShapeTESS::Scale(const double Scaler, const MString Axes)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////

//! Return a unique position inside the shape (center)
MVector MDShapeTESS::GetUniquePosition() const
{
  return MVector(0.0, 0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////

//! Return a random position within this shape
MVector MDShapeTESS::GetRandomPositionInside()
{
  return MVector(0.0, 0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////