// ESLint（flat config）。目的は **useEffect の依存配列の検査**（react-hooks）と
// TS の取りこぼしの検出。書式は Prettier 等に任せず、ここでは扱わない
// （既存コードの流儀を一括で書き換えないため）。
import js from '@eslint/js'
import tseslint from 'typescript-eslint'
import reactHooks from 'eslint-plugin-react-hooks'
import globals from 'globals'

export default tseslint.config(
  { ignores: ['dist', 'src/generated'] },
  {
    files: ['**/*.{ts,tsx}'],
    extends: [js.configs.recommended, ...tseslint.configs.recommended],
    languageOptions: { ecmaVersion: 2022, globals: globals.browser },
    plugins: { 'react-hooks': reactHooks },
    rules: {
      'react-hooks/rules-of-hooks': 'error',
      'react-hooks/exhaustive-deps': 'warn',
      // JSX の本文では全角スペースを字下げに使っている（表示のため・意図どおり）
      'no-irregular-whitespace': ['error', { skipJSXText: true }],
      '@typescript-eslint/no-unused-vars': ['error', { argsIgnorePattern: '^_', varsIgnorePattern: '^_' }],
    },
  },
)
