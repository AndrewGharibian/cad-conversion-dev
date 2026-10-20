#ifndef MGUICADConverter_H
#define MGUICADConverter_H

#include "TGFrame.h"
#include "TGButton.h"
#include "TGLabel.h"
#include "TGFileDialog.h"
#include "TString.h"

class MGUICADConverter : public TGTransientFrame {
private:
  TGTextButton* fBtnSelectFile;
  TGTextButton* fBtnConvert;
  TGTextButton* fBtnViewGeomega;
  TGLabel*      fStatusLabel;

  TString       fInputFilePath;
  TString       fOutputFilePath;

public:
  MGUICADConverter(const TGWindow* p, const TGWindow* main, UInt_t w = 400, UInt_t h = 250);
  virtual ~MGUICADConverter();

  void OnSelectFile();
  void OnConvert();
  void OnViewGeomega();

  virtual Bool_t ProcessMessage(Long_t msg, Long_t parm1, Long_t parm2) override;

  ClassDef(MGUICADConverter, 1)
};

#endif