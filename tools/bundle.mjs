// ============================================================================
// bundle.mjs: builds dist/fanjet-lab.html, one self-contained page that runs
// from anywhere (no server, no separate worker file). Three.js still comes
// from the CDN in the import map.
//
// The modules are written so this stays a text transform, not a bundler:
//   import { a, b } from './x.js'   ->  const { a, b } = __x;
//   export { a, b };                ->  return { a, b };   inside an IIFE
// scene.js is loaded lazily, so a missing WebGL or CDN never stops the numbers.
// ============================================================================
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const read = (p) => readFileSync(join(root, p), 'utf8').replace(/\r\n/g, '\n');

const scopeName = (spec) => '__' + spec.replace(/^\.\//, '').replace(/\.js$/, '');
const LOCAL_IMPORT = /^import \{([^}]*)\} from '(\.\/[\w.]+)';$/gm;

function unmodule(src) {
  return src
    .replace(LOCAL_IMPORT, (_, names, spec) => `const {${names}} = ${scopeName(spec)};`)
    .replace(/^export \{([^}]*)\};\s*$/m, (_, names) => `return {${names}};\n`)
    .replace(/\n+$/, '\n');
}

const iife = (name, src) => `const ${name} = (() => {\n${unmodule(src)}})();\n`;

function sceneLoader(src) {
  const body = unmodule(
    src
      .replace(/^import \* as (\w+) from '(three)';$/m, (_, n, spec) => `const ${n} = await import('${spec}');`)
      .replace(/^import \{([^}]*)\} from '(three\/[^']+)';$/gm, (_, names, spec) => `const {${names}} = await import('${spec}');`),
  );
  return `const __sceneLoad = async () => {\n${body}};\n`;
}

function app(src) {
  const body = unmodule(src.replace("await import('./scene.js')", 'await __sceneLoad()'));
  return `const __app = (async () => {\n${body}})();\n`;
}

const core = [
  iife('__physics', read('src/physics.js')),
  iife('__layout', read('src/layout.js')),
  iife('__flow', read('src/flow.js')),
].join('\n');

const workerSrc = core + '\n' + unmodule(read('src/particles.worker.js'));
const mainSrc = [core, sceneLoader(read('src/scene.js')), app(read('src/app.js'))].join('\n');

for (const s of [workerSrc, mainSrc]) {
  if (/^\s*(import|export)\s/m.test(s)) throw new Error('a module statement survived the transform');
  if (s.includes('</script')) throw new Error('script text contains </script');
}

let html = read('index.html');
const swap = (from, to) => {
  if (!html.includes(from)) throw new Error(`index.html is missing: ${from}`);
  html = html.replace(from, () => to);
};
swap('<link rel="stylesheet" href="src/styles.css">', `<style>\n${read('src/styles.css')}</style>`);
swap(
  '<script type="module" src="src/app.js"></script>',
  `<script type="text/plain" id="worker-src">\n${workerSrc}</script>\n<script type="module">\n${mainSrc}</script>`,
);

mkdirSync(join(root, 'dist'), { recursive: true });
const out = join(root, 'dist', 'fanjet-lab.html');
writeFileSync(out, html);
console.log(`dist/fanjet-lab.html  ${(html.length / 1024).toFixed(1)} KB`);
