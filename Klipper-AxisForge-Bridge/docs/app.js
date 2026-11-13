const printerTools = ["fluidd", "mainsail"];

function validHttpUrl(value) {
  try {
    const url = new URL(value);
    return (url.protocol === "http:" || url.protocol === "https:") && Boolean(url.hostname);
  } catch {
    return false;
  }
}

function restoreTool(name) {
  const input = document.getElementById(`${name}-url`);
  const link = document.getElementById(`${name}-open`);
  const message = document.getElementById(`${name}-message`);
  const saved = localStorage.getItem(`robot-tool-launcher.${name}`);
  if (saved && validHttpUrl(saved)) {
    input.value = saved;
    link.href = saved;
    link.hidden = false;
    message.textContent = "Address saved in this browser.";
    message.className = "message success";
  }
}

for (const name of printerTools) {
  restoreTool(name);
  document.querySelector(`form[data-printer="${name}"]`).addEventListener("submit", (event) => {
    event.preventDefault();
    const input = document.getElementById(`${name}-url`);
    const link = document.getElementById(`${name}-open`);
    const message = document.getElementById(`${name}-message`);
    const value = input.value.trim();
    if (!validHttpUrl(value)) {
      message.textContent = "Enter a full HTTP or HTTPS address.";
      message.className = "message error";
      link.hidden = true;
      return;
    }
    localStorage.setItem(`robot-tool-launcher.${name}`, value);
    link.href = value;
    link.hidden = false;
    message.textContent = "Address saved in this browser.";
    message.className = "message success";
  });
}
