
/*  
<div class="col-lg-3">
            <!-- Dropdown Card Example -->
            <div class="card shadow mb-4">
              <!-- Card Header - Dropdown -->
              <div class="card-header py-3 d-flex flex-row align-items-center justify-content-between">
                <h6 class="m-0 font-weight-bold text-primary">Chair 1</h6>
            
                <a href="#" class="btn btn-success">
                    <span class="text-white-50">
                    </span>
                    <span class="text">Split Button Success</span>
                </a>
              </div>

              <!-- Card Body -->
              <div class="card-body">
                <div class="content-row">
                  <div class="col-lg-6">
                    Current user: 
                  </div>
              
                </div>
              </div>
            </div>
          </div>

*/

function create_chair_card(content, otherinfo){
    var d1 = document.createElement("div");
    d1.className += "col-lg-3";
    content.append(d1);
}
function load_chairs(){
    var content = document.getElementById("wheelchair-content");
    $.get("http://192.168.43.122:5000/chairs", function(data) {
      console.log(data);
    });

    


}