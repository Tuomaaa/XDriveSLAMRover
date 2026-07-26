document.addEventListener("DOMContentLoaded", () => {
  const contentInfo = document.querySelector("footer [role='contentinfo'] p");
  if (!contentInfo) {
    return;
  }

  const license = document.createElement("span");
  license.className = "textbook-license";
  license.append("Textbook content is licensed under ");

  const link = document.createElement("a");
  link.href = "https://creativecommons.org/licenses/by/4.0/";
  link.rel = "license";
  link.textContent = "CC BY 4.0";

  license.append(link, ".");
  contentInfo.appendChild(license);
});
