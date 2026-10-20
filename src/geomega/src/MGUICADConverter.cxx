#include "../inc/MGUICADConverter.h"
#include <TGFileDialog.h>
#include <iostream>
#include <TG3DLine.h>
#include <TSystem.h>
#include <cstdlib>

enum EControlIDs {
  c_SelectFile = 1001,
  c_SelectDest,
  c_Convert,
  c_ViewGeomega
};

MGUICADConverter::MGUICADConverter(const TGWindow* p, const TGWindow* main, UInt_t w, UInt_t h)
  : TGTransientFrame(p, main, w, h) {

  SetCleanup(kDeepCleanup);
  TGLayoutHints* expandX = new TGLayoutHints(kLHintsExpandX, 5, 5, 5, 5);
  TGLayoutHints* leftAlign = new TGLayoutHints(kLHintsLeft, 5, 5, 5, 5);

  // 1. Source File
  fBtnSelectFile = new TGTextButton(this, "Get STEP File", c_SelectFile);
  fBtnSelectFile->Associate(this);
  AddFrame(fBtnSelectFile, expandX);

  fStatusLabel = new TGLabel(this, "No file selected");
  AddFrame(fStatusLabel, expandX);

  AddFrame(new TGHorizontal3DLine(this), expandX);

  // 2. Options (Formats)
  AddFrame(new TGLabel(this, "Output Formats (Defaults to .geo if none checked):"), leftAlign);
  TGHorizontalFrame* fFrameFormats = new TGHorizontalFrame(this);
  fCheckGeo = new TGCheckButton(fFrameFormats, ".geo");
  fCheckObj = new TGCheckButton(fFrameFormats, ".obj");
  fCheckStl = new TGCheckButton(fFrameFormats, ".stl");
  fFrameFormats->AddFrame(fCheckGeo, leftAlign);
  fFrameFormats->AddFrame(fCheckObj, leftAlign);
  fFrameFormats->AddFrame(fCheckStl, leftAlign);
  AddFrame(fFrameFormats, expandX);

  // 3. Tessellation Values
  TGHorizontalFrame* fFrameAng = new TGHorizontalFrame(this);
  fFrameAng->AddFrame(new TGLabel(fFrameAng, "Angular Deflection:"), leftAlign);
  fNumAngDef = new TGNumberEntry(fFrameAng, 0.5, 6, -1, TGNumberFormat::kNESReal);
  fFrameAng->AddFrame(fNumAngDef, leftAlign);
  AddFrame(fFrameAng, expandX);

  TGHorizontalFrame* fFrameDev = new TGHorizontalFrame(this);
  fFrameDev->AddFrame(new TGLabel(fFrameDev, "Max Deviation:"), leftAlign);
  fNumMaxDev = new TGNumberEntry(fFrameDev, 0.1, 6, -1, TGNumberFormat::kNESReal);
  fFrameDev->AddFrame(fNumMaxDev, leftAlign);
  AddFrame(fFrameDev, expandX);

  TGHorizontalFrame* fFrameVol = new TGHorizontalFrame(this);
  fFrameVol->AddFrame(new TGLabel(fFrameVol, "Min Volume:"), leftAlign);
  fNumMinVol = new TGNumberEntry(fFrameVol, 0.001, 6, -1, TGNumberFormat::kNESReal);
  fFrameVol->AddFrame(fNumMinVol, leftAlign);
  AddFrame(fFrameVol, expandX);

 // 4. File Destination
  TGHorizontalFrame* fFrameDest = new TGHorizontalFrame(this);
  fDestPath = new TGTextEntry(fFrameDest, "Destination defaults to source file directory...");
  fDestPath->SetEnabled(kFALSE);
  fFrameDest->AddFrame(fDestPath, new TGLayoutHints(kLHintsExpandX, 5, 5, 5, 5));
  
  // Change the text on this button here:
  fBtnDestBrowse = new TGTextButton(fFrameDest, "File Destination", c_SelectDest);
  
  fBtnDestBrowse->Associate(this);
  fFrameDest->AddFrame(fBtnDestBrowse, leftAlign);
  AddFrame(fFrameDest, expandX);

  // 5. Convert Button
  fBtnConvert = new TGTextButton(this, "Convert", c_Convert);
  fBtnConvert->Associate(this);
  AddFrame(fBtnConvert, expandX);

  // 6. View in Geomega Button
  fBtnViewGeomega = new TGTextButton(this, "View in Geomega", c_ViewGeomega);
  fBtnViewGeomega->Associate(this);
  fBtnViewGeomega->SetEnabled(kFALSE); // Disabled until conversion is done
  AddFrame(fBtnViewGeomega, expandX);

  MapSubwindows();
  Resize(450, 430);
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
    
    // Auto-set destination to source file's directory
    fOutputDir = gSystem->DirName(fInputFilePath);
    fDestPath->SetText(fOutputDir.Data());
    Layout();
  }
}

void MGUICADConverter::OnSelectDest() {
  static TString dir(".");
  TGFileInfo fileInfo;
  fileInfo.fIniDir = StrDup(fOutputDir.Length() > 0 ? fOutputDir.Data() : dir.Data());
  
  new TGFileDialog(gClient->GetRoot(), this, kDOpen, &fileInfo);
  if (fileInfo.fIniDir) {
    fOutputDir = fileInfo.fIniDir;
    fDestPath->SetText(fOutputDir.Data());
  }
}

void MGUICADConverter::OnConvert() {
  if (fInputFilePath.Length() == 0) {
    std::cout << "Error: No source file selected!" << std::endl;
    return;
  }

  // Handle format default logic
  bool doGeo = fCheckGeo->IsOn();
  bool doObj = fCheckObj->IsOn();
  bool doStl = fCheckStl->IsOn();
  
  if (!doGeo && !doObj && !doStl) {
    std::cout << "No format selected. Defaulting to .geo" << std::endl;
    doGeo = true;
  }

  // Grab numbers
  double angDef = fNumAngDef->GetNumber();
  double maxDev = fNumMaxDev->GetNumber();
  double minVol = fNumMinVol->GetNumber();

  // Create expected output filename (.geo) for the "View in Geomega" button
  TString baseName = gSystem->BaseName(fInputFilePath);
  if (baseName.EndsWith(".step")) baseName.ReplaceAll(".step", ".geo");
  else if (baseName.EndsWith(".stp")) baseName.ReplaceAll(".stp", ".geo");
  else baseName += ".geo";
  
  fOutputFilePath = fOutputDir + "/" + baseName;

  std::cout << "--- CONVERSION START ---" << std::endl;
  std::cout << "Source: " << fInputFilePath << std::endl;
  std::cout << "Destination: " << fOutputDir << std::endl;
  std::cout << "Target .geo: " << fOutputFilePath << std::endl;
  std::cout << "Formats - geo: " << doGeo << ", obj: " << doObj << ", stl: " << doStl << std::endl;
  std::cout << "Params - AngDef: " << angDef << ", MaxDev: " << maxDev << ", MinVol: " << minVol << std::endl;
  std::cout << "------------------------" << std::endl;

  // Unlock the View in Geomega button
  fBtnViewGeomega->SetEnabled(kTRUE);
  Layout();
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
            case c_SelectDest:  OnSelectDest(); break;
            case c_Convert:     OnConvert(); break;
            case c_ViewGeomega: OnViewGeomega(); break;
          }
          break;
      }
      break;
  }
  return kTRUE;
}