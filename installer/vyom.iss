; Inno Setup script for the Vyom desktop app.
; Build the exe first (PyInstaller, desktop_app.spec), then:
;   "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" installer\vyom.iss
; Output: dist\VyomSetup-<version>.exe
;
; User data (scenarios, run history, imported aero decks, license key) lives
; in %APPDATA%\Vyom and is deliberately left in place on uninstall.

#define AppName "Vyom"
#define AppVersion "0.3.0"
#define AppPublisher "Vyom"
#define AppExe "Vyom.exe"

[Setup]
AppId={{6E7B3C1A-4F2D-4B8E-9A61-B0057BACC0DE}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
OutputDir=..\dist
OutputBaseFilename=VyomSetup-{#AppVersion}
SetupIconFile=..\static\app_icon.ico
UninstallDisplayIcon={app}\{#AppExe}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
ArchitecturesInstallIn64BitMode=x64compatible

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "..\dist\{#AppExe}"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
