// Dummy user challan info
const challan = {
  userId: "User123",
  location: "Jayanagar Bangalore Park",
  status: localStorage.getItem("penaltyStatus") || "Unpaid"
};

// Handle Pay Now button (index.html)
const payBtn = document.getElementById("payBtn");
if (payBtn) {
  payBtn.addEventListener("click", () => {
    challan.status = "Paid";
    localStorage.setItem("penaltyStatus", "Paid");
    alert("✅ Payment Successful! Thank you.");
    window.location.reload();
  });
}

// Fill Admin Table (admin.html)
const userTable = document.getElementById("userTable");
if (userTable) {
  const row = document.createElement("tr");
  row.innerHTML = `
    <td>${challan.userId}</td>
    <td>${challan.location}</td>
    <td style="color:${challan.status === "Paid" ? "green" : "red"};">
      ${challan.status}
    </td>
  `;
  userTable.appendChild(row);
}
