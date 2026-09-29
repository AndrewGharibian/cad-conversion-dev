$tokens = ((Select-String -Path '.\examples\box_tessellated.geo.setup' -Pattern '^MeshBox\.Shape ').Line -split '\s+');
$vertexCount = [int]$tokens[2]
$triangleCountIndex = 3 + 3 * $vertexCount
$triangleCount = [int]$tokens[$triangleCountIndex]
$vertices = @()
for ($i = 0; $i -lt $vertexCount; $i++) {
    $base = 3 + 3 * $i
    $vertices += ,@([double]$tokens[$base], [double]$tokens[$base + 1], [double]$tokens[$base + 2])
}
$sum = 0.0
for ($i = 0; $i -lt $triangleCount; $i++) {
    $base = $triangleCountIndex + 1 + 3 * $i
    $a = $vertices[[int]$tokens[$base]]
    $b = $vertices[[int]$tokens[$base + 1]]
    $c = $vertices[[int]$tokens[$base + 2]]
    $sum += $a[0] * ($b[1] * $c[2] - $b[2] * $c[1]) + $a[1] * ($b[2] * $c[0] - $b[0] * $c[2]) + $a[2] * ($b[0] * $c[1] - $b[1] * $c[0])
}
$volume = $sum / 6.0
if ([math]::Abs($volume - 8000.0) -gt 0.001) { throw "Unexpected signed mesh volume: $volume" }
"Outward winding verified; signed volume: $volume cm^3."
