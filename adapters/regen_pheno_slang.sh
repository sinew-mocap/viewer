#!/usr/bin/env bash
# Regenerate the phenotype Slang→CPU sources the viewer compiles: emit the kernels
# from Lean, run slangc -target cpp, vendor the slang C++ prelude, and rewrite the
# generated #include to the vendored copy so the output builds with a plain C++
# compiler (no slangc / lake at viewer build time).  Run after editing
# spec/Sinew/SlangCodegen/Pheno.lean.
#
#   viz_native/regen_pheno_slang.sh
set -e
here="$(cd "$(dirname "$0")" && pwd)"
spec="$here/../spec"
gen="$here/gen"
rt="$here/slang_rt"
mkdir -p "$gen" "$rt"

(cd "$spec" && lake exe emit_shaders slang)

emit() { slangc "$spec/slang/$1.slang" -entry "$2" -stage compute -target cpp -o "$gen/$1.gen.cpp"; }
emit pheno_blendshape   blendshape
emit pheno_bary_tet     bary_tet
emit pheno_bary_gather  bary_gather
emit pheno_rbf          rbf
emit pheno_skeleton_fit skeleton_fit
emit pheno_se3_inverse  se3_inverse

# LBS kernel as SPIR-V for the viewer's per-frame Vulkan compute host.
slangc "$spec/slang/lbs.slang" -entry lbs -stage compute -fvk-use-entrypoint-name \
	-target spirv -o "$gen/lbs.spv"

# Vendor the (self-contained) slang C++ prelude chain from wherever slangc baked it.
prelude="$(sed -n '1s/.*"\(.*\)".*/\1/p' "$gen/pheno_blendshape.gen.cpp")"
pdir="$(dirname "$prelude")"
for h in slang-cpp-prelude.h slang-llvm.h slang-cpp-scalar-intrinsics.h \
         slang-cpp-types.h slang-cpp-types-core.h; do
	cp "$pdir/$h" "$rt/$h"
done

# Point each generated file at the vendored prelude (relative to viz_native, -I).
for g in "$gen"/pheno_*.gen.cpp; do
	sed -i '1s#".*slang-cpp-prelude.h"#"slang_rt/slang-cpp-prelude.h"#' "$g"
done
echo "regenerated $gen/*.gen.cpp + vendored prelude in $rt"
