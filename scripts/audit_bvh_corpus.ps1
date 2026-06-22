param(
    [string]$BvhDir = "BVH",
    [string]$Cli = "build/Debug/pmg_cli.exe",
    [string]$OutputCsv = "build/compatible_bvh_corpus_audit.csv",
    [string]$OutputMd = "build/compatible_bvh_corpus_audit.md"
)

$ErrorActionPreference = "Stop"
$files = Get-ChildItem $BvhDir -Filter *.bvh | Sort-Object Name
$rows = foreach ($file in $files) {
    $summary = & $Cli --bvh $file.FullName
    if ($LASTEXITCODE -ne 0) { throw "Failed to inspect $($file.FullName)" }
    $name = $file.BaseName.ToLowerInvariant()
    $category = if ($name -match "jog|run") { "RUN_COMPATIBLE" }
        elseif ($name -match "start|stop") { "STOP_START_COMPATIBLE" }
        elseif ($name -match "walk|strut|sneak") { "WALK_COMPATIBLE" }
        elseif ($name -match "aboutface") { "TURN_IN_PLACE_COMPATIBLE" }
        else { "ACTION_COMPATIBLE" }
    $cyclic = $name -match "loop|curve|spiral|walk|jog|strut|sneak|still"
    [pscustomobject]@{
        file = $file.Name
        joints = [int](($summary | Select-String '^joints:').Line.Split(':')[1])
        fps = [double](($summary | Select-String '^fps:').Line.Split(':')[1])
        frames = [int](($summary | Select-String '^frames:').Line.Split(':')[1])
        category = $category
        cyclic_candidate = $cyclic
        usable_for_second_node = $category -eq "RUN_COMPATIBLE" -and $cyclic -and $name -notmatch "to"
    }
}

$rows | Export-Csv $OutputCsv -NoTypeInformation
$runCandidates = @($rows | Where-Object usable_for_second_node)
$decision = if ($runCandidates.Count -ge 3) { "READY_FOR_SECOND_NODE" } else { "ONLY_SINGLE_NODE_IS_JUSTIFIED" }
@"
# Compatible BVH Corpus Audit

## Decision

``$decision``

## Evidence

- Files scanned: $($rows.Count).
- Common acquisition: $((@($rows.fps | Sort-Object -Unique) -join ', ')) fps; $((@($rows.joints | Sort-Object -Unique) -join ', ')) joints.
- Cyclic run candidates: $($runCandidates.Count) ($((@($runCandidates.file) -join ', '))).
- `walkToJog.bvh` is a transition clip, not an independent cyclic run anchor.

## Limitation

Filename classification only identifies candidates. A second node still requires at least three coherent cyclic run clips plus registration, root-motion, and contact audits. Current corpus does not supply that group.
"@ | Set-Content $OutputMd

Write-Output $decision
