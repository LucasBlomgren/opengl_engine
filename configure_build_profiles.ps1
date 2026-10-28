# Configure Debug / Dev / Release for the opengl_engine Visual Studio C++ project.
# Run from the repository root while Visual Studio is closed.
# Existing O2 configuration is renamed to Dev. Only x64 build flags are updated.
param(
    [string]$Root = (Get-Location).Path
)

$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path -LiteralPath $Root).Path

$projects = @(Get-ChildItem -LiteralPath $Root -Recurse -File -Filter '*.vcxproj' | Where-Object { $_.FullName -notmatch '[\\/](build|out|\.vs)[\\/]' })
if ($projects.Count -ne 1) {
    throw "Expected one .vcxproj under '$Root', found $($projects.Count). Pass -Root pointing to the engine directory containing one project."
}
$project = $projects[0]

$solutions = @(
    Get-ChildItem -LiteralPath $Root -File | Where-Object { $_.Extension -in @('.sln', '.slnx') }
)
if ($solutions.Count -gt 1) {
    throw "Multiple .sln/.slnx files were found in '$Root'. Keep one solution in the root before running."
}
if ($solutions.Count -eq 0) {
    throw "No .sln or .slnx in '$Root'. Run from the solution folder."
}
$solution = $solutions[0]

$xml = New-Object System.Xml.XmlDocument
$xml.PreserveWhitespace = $true
$xml.Load($project.FullName)

# Only rename the actual build configuration O2. Do not replace compiler flag /O2.
$configurationItems = @($xml.SelectNodes("/*[local-name()='Project']/*[local-name()='ItemGroup'][@Label='ProjectConfigurations']/*[local-name()='ProjectConfiguration']"))
$names = @($configurationItems | ForEach-Object { $_.GetAttribute('Include') })
foreach ($required in @('Debug|x64', 'Release|x64')) {
    if ($names -notcontains $required) { throw "The project does not have $required." }
}
if ($names -notcontains 'Dev|x64' -and $names -notcontains 'O2|x64') {
    throw 'Neither Dev|x64 nor O2|x64 exists. This script expects the O2 configuration shown in your project.'
}
if ($names -contains 'Dev|x64' -and $names -contains 'O2|x64') {
    throw 'Both Dev and O2 exist. Rename/remove O2 manually to avoid ambiguities before running.'
}

if ($names -contains 'O2|x64') {
    foreach ($node in $xml.SelectNodes('//*[@*]')) {
        foreach ($attr in $node.Attributes) {
            if ($attr.Value.Contains('O2|')) {
                $attr.Value = $attr.Value.Replace('O2|', 'Dev|')
            }
        }
    }
    foreach ($item in $configurationItems) {
        $nameNode = $item.SelectSingleNode("./*[local-name()='Configuration']")
        if ($null -ne $nameNode -and $nameNode.InnerText -eq 'O2') {
            $nameNode.InnerText = 'Dev'
        }
    }
}

function Set-XmlChild([System.Xml.XmlNode]$parent, [string]$name, [string]$value) {
    $child = $parent.SelectSingleNode("./*[local-name()='$name']")
    if ($null -eq $child) {
        $child = $xml.CreateElement($name, $xml.DocumentElement.NamespaceURI)
        [void]$parent.AppendChild($child)
    }
    $child.InnerText = $value
}

function Get-ConfigGroup([string]$tag, [string]$configuration, [string]$label = '') {
    $match = "'$($configuration)|x64'"
    $groups = @($xml.DocumentElement.ChildNodes | Where-Object {
        $_ -is [System.Xml.XmlElement] -and $_.LocalName -eq $tag -and $_.GetAttribute('Condition').Contains($match) -and
        ($label -eq '' -or $_.GetAttribute('Label') -eq $label)
    })
    if ($groups.Count -ne 1) {
        throw "Expected one $tag group for ${configuration}|x64 (label '$label'); got $($groups.Count)."
    }
    return $groups[0]
}

function Get-OrCreateChild([System.Xml.XmlNode]$parent, [string]$tag) {
    $child = $parent.SelectSingleNode("./*[local-name()='$tag']")
    if ($null -eq $child) {
        $child = $xml.CreateElement($tag, $xml.DocumentElement.NamespaceURI)
        [void]$parent.AppendChild($child)
    }
    return $child
}

function Set-Macros([System.Xml.XmlNode]$cl, [string]$cfg) {
    $current = $cl.SelectSingleNode("./*[local-name()='PreprocessorDefinitions']")
    $old = if ($null -ne $current) { $current.InnerText } else { '%(PreprocessorDefinitions)' }
    $macros = @($old.Split(';') | ForEach-Object { $_.Trim() } | Where-Object {
        $_ -and $_ -notin @('NDEBUG', '_DEBUG', 'PHYSICS_ENABLE_LOGGING', '%(PreprocessorDefinitions)')
    })
    # Keep all existing application macros. Remove logging in Release only.
    if ($cfg -ne 'Release' -and ($old.Split(';') -contains 'PHYSICS_ENABLE_LOGGING')) {
        $macros += 'PHYSICS_ENABLE_LOGGING'
    }
    if ($cfg -eq 'Debug') { $macros += '_DEBUG' }
    if ($cfg -eq 'Release') { $macros += 'NDEBUG' }
    $macros += '%(PreprocessorDefinitions)'
    Set-XmlChild $cl 'PreprocessorDefinitions' (($macros | Select-Object -Unique) -join ';')
}

# Keep includes, library references, language standard, PCH and per-file overrides untouched.
$profiles = @(
    @{ Name = 'Debug';   Optimization = 'Disabled'; Inline = 'Disabled';           Intrinsics = 'false'; WPO = 'false'; FL = 'false'; SizeSpeed = 'Neither'; LinkInc = 'true';  LTCG = 'Default';                   Ref = 'false'; ICF = 'false'; DebugLib = 'true'  },
    @{ Name = 'Dev';     Optimization = 'MinSpace'; Inline = 'OnlyExplicitInline'; Intrinsics = 'true';  WPO = 'false'; FL = 'true';  SizeSpeed = 'Size';    LinkInc = 'true';  LTCG = 'Default';                   Ref = 'false'; ICF = 'false'; DebugLib = 'false' },
    @{ Name = 'Release'; Optimization = 'MaxSpeed'; Inline = 'AnySuitable';        Intrinsics = 'true';  WPO = 'true';  FL = 'true';  SizeSpeed = 'Speed';   LinkInc = 'false'; LTCG = 'UseLinkTimeCodeGeneration'; Ref = 'true';  ICF = 'true';  DebugLib = 'false' }
)

foreach ($p in $profiles) {
    $cfg = $p.Name
    $pg = Get-ConfigGroup 'PropertyGroup' $cfg 'Configuration'
    Set-XmlChild $pg 'UseDebugLibraries' $p.DebugLib
    Set-XmlChild $pg 'WholeProgramOptimization' $p.WPO

    # This property group is the x64 linker general settings group (NOT Label=Configuration).
    $otherPGs = @($xml.DocumentElement.ChildNodes | Where-Object {
        $_ -is [System.Xml.XmlElement] -and $_.LocalName -eq 'PropertyGroup' -and $_.GetAttribute('Condition').Contains("'$cfg|x64'") -and $_.GetAttribute('Label') -ne 'Configuration'
    })
    if ($otherPGs.Count -lt 1) { throw "Missing x64 property group for $cfg." }
    $otherPG = $otherPGs[0]
    Set-XmlChild $otherPG 'LinkIncremental' $p.LinkInc

    $idg = Get-ConfigGroup 'ItemDefinitionGroup' $cfg
    $cl = Get-OrCreateChild $idg 'ClCompile'
    $link = Get-OrCreateChild $idg 'Link'

    Set-XmlChild $cl 'Optimization' $p.Optimization
    Set-XmlChild $cl 'WholeProgramOptimization' $p.WPO
    Set-XmlChild $cl 'InlineFunctionExpansion' $p.Inline
    Set-XmlChild $cl 'IntrinsicFunctions' $p.Intrinsics
    Set-XmlChild $cl 'FunctionLevelLinking' $p.FL
    Set-XmlChild $cl 'FavorSizeOrSpeed' $p.SizeSpeed
    Set-XmlChild $cl 'MultiProcessorCompilation' 'true'
    Set-XmlChild $cl 'DebugInformationFormat' 'ProgramDatabase'
    Set-XmlChild $cl 'FloatingPointModel' 'Precise'
    Set-Macros $cl $cfg

    Set-XmlChild $link 'LinkTimeCodeGeneration' $p.LTCG
    Set-XmlChild $link 'OptimizeReferences' $p.Ref
    Set-XmlChild $link 'EnableCOMDATFolding' $p.ICF
    Set-XmlChild $link 'GenerateDebugInformation' 'true'
}

# Dev must use its own build folders. Project default paths normally use $(Configuration),
# but a custom path copied from O2 may be hard-coded; replace those if present.
foreach ($node in $xml.SelectNodes('//*[local-name()="PropertyGroup" or local-name()="ClCompile" or local-name()="Link"]/*')) {
    if ($null -ne $node.ParentNode -and $node.ParentNode.GetAttribute('Condition').Contains("'Dev|")) {
        if ($node.LocalName -in @('IntDir','OutDir','TargetName','ProgramDataBaseFileName')) {
            $node.InnerText = $node.InnerText.Replace('O2', 'Dev')
        }
    }
}

$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$projectBackup = "$($project.FullName).backup-$timestamp"
$solutionBackup = "$($solution.FullName).backup-$timestamp"
Copy-Item -LiteralPath $project.FullName -Destination $projectBackup
Copy-Item -LiteralPath $solution.FullName -Destination $solutionBackup

try {
    # Update the IDE dropdown: O2 -> Dev in both .sln and .slnx formats.
    $slnText = [System.IO.File]::ReadAllText($solution.FullName)
    if ($solution.Extension -eq '.sln') {
        $slnText = $slnText.Replace('O2|', 'Dev|')
    } else {
        $slnText = $slnText.Replace('Name="O2"', 'Name="Dev"')
        $slnText = $slnText.Replace('Configuration="O2"', 'Configuration="Dev"')
        $slnText = $slnText.Replace('O2|', 'Dev|')
    }
    $xml.Save($project.FullName)
    [System.IO.File]::WriteAllText($solution.FullName, $slnText, (New-Object System.Text.UTF8Encoding($true)))
} catch {
    Copy-Item -LiteralPath $projectBackup -Destination $project.FullName -Force
    Copy-Item -LiteralPath $solutionBackup -Destination $solution.FullName -Force
    throw
}

Write-Host ''
Write-Host 'Configured build profiles (x64):' -ForegroundColor Green
Write-Host '  Debug:   /Od, /Zi, /INCREMENTAL, no /GL/LTCG'
Write-Host '  Dev:     /O1 /Ob1, /Zi, /INCREMENTAL, no /GL/LTCG'
Write-Host '  Release: /O2 /GL /LTCG /OPT:REF /OPT:ICF, /INCREMENTAL:NO'
Write-Host "Project: $($project.FullName)"
Write-Host "Solution: $($solution.FullName)"
Write-Host "Backups: $projectBackup and $solutionBackup"
Write-Host 'Reopen Visual Studio and run Rebuild for each profile once.'
