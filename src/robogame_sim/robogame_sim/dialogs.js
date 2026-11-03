// The RoboGame Gamemaster's dialogs, the same on every tab: a question
// with OK and Cancel, or one that asks for a name. Enter is OK and Escape
// is Cancel, and while one is open the page's own keys are off: check
// dialogOpen() first in a keydown handler.
"use strict";

// Both pages have these colours, as robot_console.html names them.
document.head.append(Object.assign(document.createElement("style"), {
  textContent: `
  dialog { background: var(--panel); color: var(--ink); min-width: 280px;
           max-width: 440px; border: 1px solid var(--line);
           border-radius: 8px; padding: 16px; }
  dialog::backdrop { background: rgb(0 0 0 / .35); }
  dialog h2 { margin-top: 0; }
  dialog p { margin: 0 0 10px; }
  dialog input, dialog select { width: 100%; box-sizing: border-box; }
  .dialog-buttons { display: flex; gap: 8px; justify-content: flex-end;
                    margin-top: 14px; }
  button.primary.danger { background: var(--bad); border-color: var(--bad);
                          color: #fff; }`,
}));

const dialogOpen = () => !!document.querySelector("dialog[open]");

// Enter in any field of a form dialog, a select too, presses its OK.
document.addEventListener("keydown", event => {
  const dialog = event.target.closest && event.target.closest("dialog[open]");
  if (!dialog || event.key !== "Enter" || event.target.tagName === "BUTTON"
      || event.target.tagName === "TEXTAREA") return;
  const form = dialog.querySelector("form");
  if (!form) return;
  event.preventDefault();
  const ok = form.querySelector("button.primary");
  if (form.reportValidity()) form.requestSubmit(ok || undefined);
});

// Shows the question, and an input with value if it is not undefined, and
// resolves to the input's text, true for OK with no input, or null.
function ask(message, value, okText = "OK", danger = false) {
  const dialog = document.createElement("dialog");
  dialog.className = "ask";
  const form = document.createElement("form");
  form.method = "dialog";
  const p = document.createElement("p");
  p.textContent = message;
  form.append(p);
  let input = null;
  if (value !== undefined) {
    input = document.createElement("input");
    input.value = value;
    input.required = true;
    form.append(input);
  }
  const buttons = document.createElement("div");
  buttons.className = "dialog-buttons";
  const cancel = document.createElement("button");
  cancel.value = "cancel";
  cancel.formNoValidate = true;
  cancel.textContent = "Cancel";
  const ok = document.createElement("button");
  ok.value = "ok";
  ok.className = danger ? "primary danger" : "primary";
  ok.textContent = okText;
  buttons.append(cancel, ok);
  form.append(buttons);
  dialog.append(form);
  document.body.append(dialog);
  return new Promise(resolve => {
    dialog.addEventListener("close", () => {
      const yes = dialog.returnValue === "ok";
      resolve(yes ? (input ? input.value : true) : null);
      dialog.remove();
    }, { once: true });
    dialog.showModal();
    if (input) input.select(); else ok.focus();
  });
}

const askText = (message, value, okText) => ask(message, value || "",
                                                okText);
const askYes = (message, okText, danger) => ask(message, undefined, okText,
                                                danger);
