#include "../inc/MGUICADConverter.h"
#include <iostream>
#include <cstdlib>

enum EControlIDs {
  c_SelectFile = 1001,
  c_Convert,
  c_ViewGeomega
};

MGUICADConverter::MGUICADConverter(const TGWindow* p, const TGWindow* main, UInt_t w, UInt_t h)
  : TGTransientFrame(p, main, w, h) {

  SetCleanup(kDeepCleanup);

  // Status / File Path Label
  fStatusLabel = new TGLabel(this, "No file selected");
  AddFrame(fStatusLabel, new TGLayoutHints(kLHintsExpandX, 10, 10, 10, 5));

  // 1. Select File Button
  fBtnSelectFile = new TGTextButton(this, "Select CAD File", c_SelectFile);
  fBtnSelectFile->Associate(this);
  AddFrame(fBtnSelectFile, new TGLayoutHints(kLHintsExpandX, 10, 10, 5, 5));

  // 2. Convert Button
  fBtnConvert = new TGTextButton(this, "Convert to .geo", c_Convert);
  fBtnConvert->Associate(this);
  fBtnConvert->SetEnabled(kFALSE); // Disabled until file is chosen
  AddFrame(fBtnConvert, new TGLayoutHints(kLHintsExpandX, 10, 10, 5, 5));

  // 3. View in Geomega Button
  fBtnViewGeomega = new TGTextButton(this, "View in Geomega", c_ViewGeomega);
  fBtnViewGeomega->Associate(this);
  fBtnViewGeomega->SetEnabled(kFALSE); // Disabled until conversion is done
  AddFrame(fBtnViewGeomega, new TGLayoutHints(kLHintsExpandX, 10, 10, 5, 10));

  MapSubwindows();
  Resize(GetDefaultSize());
  MapWindow();
}

MGUICADConverter::~MGUICADConverter() {}

void MGUICADConverter::OnSelectFile() {
  static TString dir(".");
  TGFileInfo fileInfo;
  fileInfo.fIniDir = StrDup(dir);
  
  new TGFileDialog(gClient->GetRoot(), this, kFDOpen, &fileInfo);
  if (fileInfo.fFilename) {
    fInputFilePath = fileInfo.fFilename;
    fStatusLabel->SetText(Form("Selected: %s", fInputFilePath.Data()));
    fBtnConvert->SetEnabled(kTRUE);
    Layout();
  }
}

void MGUICADConverter::OnConvert() {
  static TString dir(".");
  TGFileInfo fileInfo;
  fileInfo.fIniDir = StrDup(dir);
  
  new TGFileDialog(gClient->GetRoot(), this, kFDSave, &fileInfo);
  if (fileInfo.fFilename) {
    fOutputFilePath = fileInfo.fFilename;
    if (!fOutputFilePath.EndsWith(".geo")) {
      fOutputFilePath += ".geo";
    }

    fStatusLabel->SetText("Converting CAD file...");
    Layout();

    // TODO: Insert actual CAD conversion engine logic here
    std::cout << "Converting " << fInputFilePath << " -> " << fOutputFilePath << std::endl;

    fStatusLabel->SetText(Form("Done! Saved to: %s", fOutputFilePath.Data()));
    fBtnViewGeomega->SetEnabled(kTRUE);
    Layout();
  }
}

void MGUICADConverter::OnViewGeomega() {
  if (fOutputFilePath.Length() > 0) {
    TString cmd = Form("geomega -g %s &", fOutputFilePath.Data());
    std::cout << "Launching: " << cmd << std::endl;
    system(cmd.Data());
  }
}

Bool_t MGUICADConverter::ProcessMessage(Long_t msg, Long_t parm1, Long_t parm2) {
  switch (GET_MSG(msg)) {
    case kC_COMMAND:
      switch (GET_SUBMSG(msg)) {
        case kCM_BUTTON:
          switch (parm1) {
            case c_SelectFile:  OnSelectFile(); break;
            case c_Convert:     OnConvert(); break;
            case c_ViewGeomega: OnViewGeomega(); break;
          }
          break;
      }
      break;
  }
  return kTRUE;
}