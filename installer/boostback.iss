; Inno Setup script for the Boostback desktop app.
; Build the exe first (PyInstaller, desktop_app.spec), then:
;   "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" installer\boostback.iss
; Output: dist\BoostbackSetup-<version>.exe
;
; User data (scenarios, run history, imported aero decks, license key) lives
; in %APPDATA%\Boostback and is deliberately left in place on uninstall.

#define AppName "Boostback"
#define AppVersion "0.3.0"
#define AppPublisher "Boostback"
#define AppExe "Boostback.exe"

[Setup]
AppId={{6E7B3C1A-4F2D-4B8E-9A61-B0057BACC0DE}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
OutputDir=..\dist
OutputBaseFilename=BoostbackSetup-{#AppVersion}
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
