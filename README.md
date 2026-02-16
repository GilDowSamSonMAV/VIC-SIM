# VIC Simulator

A web-based simulator for the VIC (Visual Computer) educational assembly language.

## Features

- **Full VIC Assembly Language** — write symbolic assembly with labels and variables
- **Machine Code Mode** — enter raw numeric instructions
- **Step-by-step Execution** — walk through programs one instruction at a time
- **Visual Memory Display** — see all 100 memory cells with highlighting for PC, changed values, and variables
- **CPU Registers** — real-time display of PC, IR, and Data Register with decoded instructions
- **Built-in Examples** — modulo, multiply, abs, max, countdown, sum, division, fibonacci
- **Questions Sidebar** — load test questions with multiple-choice and free-text support
- **Keyboard Shortcuts** — Ctrl+Enter (assemble), F5 (run), F10 (step), Esc (pause)

## Deploy to GitHub Pages

1. Create a new repository on GitHub
2. Push this folder to the repo
3. Go to Settings → Pages → Source: Deploy from branch → `main` / `root`
4. Your simulator will be live at `https://yourusername.github.io/repo-name/`

## Loading Questions

Call `loadQuestions()` from the browser console or add to the HTML:

```javascript
loadQuestions([
  {
    type: 'multiple-choice',
    question: 'What does the `read` command do?',
    options: ['D = input', 'output = D', 'D = 0', 'stop'],
    correct: 0,
    explanation: 'read takes a number from input and stores it in D'
  },
  {
    type: 'free-text',
    question: 'What is the output of modulo program with input 15, 4?',
    correct: '3',
    explanation: '15 mod 4 = 3'
  }
]);
```

## VIC Instruction Set

| Command | Meaning | Machine Code |
|---------|---------|-------------|
| read | D = input | 1xx |
| write | output = D | 2xx |
| load x | D = mem[x] | 3xx |
| store x | mem[x] = D | 4xx |
| add x | D = D + mem[x] | 5xx |
| sub x | D = D - mem[x] | 6xx |
| goto LABEL | jump to LABEL | 7xx |
| gotoz LABEL | if D==0 goto LABEL | 8xx |
| gotop LABEL | if D>0 goto LABEL | 9xx |
| stop | halt execution | 000 |

Memory cells 98 and 99 are predefined as `zero` (0) and `one` (1).
