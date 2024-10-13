let elementsList = [];

function save_names(td) {
  var aElement = td.querySelector("a");
  if (aElement) {
    var text = aElement.textContent.trim();
    elementsList.push(text);
  } else {
    elementsList.push("No a element found");
  }
}

async function navigateToNextPage(next_page, page) {
  window.location.href = next_page;
  this_page = next_page;

  // Wait for the new page to load
  await waitForElement("#user-index-participants-table"); // Adjust the selector as needed

  console.log("end this page: " + page);
}

function waitForElement(selector) {
  return new Promise((resolve, reject) => {
    const observer = new MutationObserver((mutationsList, observer) => {
      if (document.querySelector(selector)) {
        observer.disconnect();
        resolve();
      }
    });

    observer.observe(document.body, { childList: true, subtree: true });
  });
}

for (let page = 1; page <= 14; page++) {
  next_page =
    "https://moodle.uni-due.de/user/index.php?contextid=4046857&id=48250&perpage=20&page=" +toString(page);
  console.log("start this page: " + page);
  console.log("page url: " + next_page);
    for (let c = 0; c <= 20; c++) {
    // Adjust the range as needed
    let ids = "user-index-participants-48250_r" + c;
    let selector = 'td[id^="' + ids + '_c1"]';
    var tdElements = document.querySelectorAll(selector);

    tdElements.forEach(save_names);
  }

  if (page < 12) {
    navigateToNextPage(next_page, page);
  }
}
// Create a Blob from the elementsList array
let blob = new Blob([elementsList.join('\n')], { type: 'text/plain' });

// Create a link element
let link = document.createElement('a');

// Set the download attribute with a filename
link.download = 'elementsList.txt';

// Create a URL for the Blob and set it as the href attribute
link.href = window.URL.createObjectURL(blob);

// Append the link to the body
document.body.appendChild(link);

// Programmatically click the link to trigger the download
link.click();

// Remove the link from the document
document.body.removeChild(link);
