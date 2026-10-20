#ifndef MGUICADConverter_H
#define MGUICADConverter_H

#include <TGFrame.h>
#include <TGLabel.h>
#include <TGButton.h>
#include <TString.h>
#include <TGNumberEntry.h>
#include <TGTextEntry.h>

class MGUICADConverter : public TGTransientFrame {
private:
  TGLabel* fStatusLabel;
  TGTextButton* fBtnSelectFile;
  TGTextButton* fBtnConvert;
  TGTextButton* fBtnViewGeomega;
  
  TGCheckButton* fCheckGeo;
  TGCheckButton* fCheckObj;
  TGCheckButton* fCheckStl;

  TGNumberEntry* fNumAngDef;
  TGNumberEntry* fNumMaxDev;
  TGNumberEntry* fNumMinVol;

  TGTextEntry* fDestPath;
  TGTextButton* fBtnDestBrowse;

  TString fInputFilePath;
  TString fOutputDir;
  TString fOutputFilePath;

public:
  MGUICADConverter(const TGWindow* p, const TGWindow* main, UInt_t w, UInt_t h);
  virtual ~MGUICADConverter();

  void OnSelectFile();
  void OnSelectDest();
  void OnConvert();
  void OnViewGeomega();

  virtual Bool_t ProcessMessage(Long_t msg, Long_t parm1, Long_t parm2) override;

  ClassDef(MGUICADConverter, 1)
};

#endif