/*
 * MDShapeTESS.h
 *
 * Copyright (C) by Andreas Zoglauer.
 * All rights reserved.
 *
 * Please see the source-file for the copyright-notice.
 *
 */

#ifndef __MDShapeTESS__
#define __MDShapeTESS__

////////////////////////////////////////////////////////////////////////////////

// ROOT libs:
class TGeoTessellated;

// MEGAlib libs:
#include "MGlobal.h"
#include "MString.h"
#include "MVector.h"
#include "MDShape.h"

// Forward declarations:

////////////////////////////////////////////////////////////////////////////////

//! Class representing a tessellated solid shape (e.g. CAD / STL mesh)
class MDShapeTESS : public MDShape
{
  // public interface:
 public:
  //! Default constructor
  MDShapeTESS(const MString& Name);
  //! Default destructor
  virtual ~MDShapeTESS();
  
  //! Set the shape mesh file
  bool Set(const MString& FileName);
  
  //! Validate the data and create the shape 
  bool Validate();  
  
  //! Parse some tokenized text
  bool Parse(const MTokenizer& Tokenizer, const MDDebugInfo& Info);

  virtual bool IsInside(const MVector& Pos, const double Tolerance = 0, const bool PreferOutside = false);

  virtual double DistanceOutsideIn(const MVector& Pos, const MVector& Dir, double Tolerance = 0);
  
  MString ToString();
  MString GetGeomega() const;

  MString GetFileName() const { return m_FileName; }

  double GetVolume();

  //! Scale the axes given in Axes by a factor Scaler
  virtual bool Scale(const double Scaler, const MString Axes = "XYZ");

  //! Return a unique position within the volume of the detector (center if possible)
  virtual MVector GetUniquePosition() const;

  //! Return a random position within this volume
  virtual MVector GetRandomPositionInside(); 

  // protected methods:
 protected:

  // private methods:
 private:

  // protected members:
 protected:

  // private members:
 private:
  MString m_FileName;


#ifdef ___CLING___
 public:
  ClassDef(MDShapeTESS, 0) // Tessellated shape
#endif

};

#endif

////////////////////////////////////////////////////////////////////////////////