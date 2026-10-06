$ErrorActionPreference = "Stop"

function Invoke-PythonStep {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)

    & python @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "python $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

Invoke-PythonStep @("get_data.py", "--drives", "0048", "0005", "0052")
Invoke-PythonStep @("run_benchmark.py")
Invoke-PythonStep @("validate_holdout.py", "--drives", "0052", "--label", "holdout")
Invoke-PythonStep @("demo.py")
