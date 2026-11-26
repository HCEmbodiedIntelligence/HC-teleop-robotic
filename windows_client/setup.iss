[Setup]
AppId=HC.Teleop.Workstation
AppName=HC 遥操作
AppVersion=0.1.0
DefaultDirName={localappdata}\Programs\HC-Teleop
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
DefaultGroupName=HC 遥操作
DisableProgramGroupPage=yes
UninstallDisplayIcon={app}\HC-Teleop.exe
OutputDir={#Artifacts}
OutputBaseFilename=HC-Teleop-Setup
Compression=lzma2/fast
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes

[Files]
Source: "{#Artifacts}\HC-Teleop.exe"; DestDir: "{app}"; Flags: ignoreversion

[Tasks]
Name: "desktopicon"; Description: "创建桌面快捷方式"

[Icons]
Name: "{group}\HC 遥操作"; Filename: "{app}\HC-Teleop.exe"
Name: "{userdesktop}\HC 遥操作"; Filename: "{app}\HC-Teleop.exe"; Tasks: desktopicon
