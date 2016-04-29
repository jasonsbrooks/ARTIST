function hideElements() {
  if ($(window).width() < 932) {
    if ($(window).width() < 534) {
      $(".documentation").hide();
    } else {
      $(".documentation").show();
    }
    $(".toHide").hide();
  } else {
    $(".toHide").show();
  }
}

$(document).ready(function() {
  hideElements();

  $(window).resize(function() {
    hideElements();
  });

  var scroll_start = 0;
  $('#logo').hide();
  var offset = $(window).height() / 3;
  $(document).scroll(function() { 
     scroll_start = $(this).scrollTop();
     if(scroll_start > offset) {
         $('.navbar').addClass('scrolled');
         $('#logo').show();
         $('#large-logo').fadeOut();
      } else {
         $('.navbar').removeClass('scrolled');
         $('#logo').hide();
         $('#large-logo').fadeIn();
      }
  });

  $(document).on('click','.scrollToDiv', function(event) {
      event.preventDefault();
      var target = "#" + this.getAttribute('data-target');
      $('html, body').animate({
          scrollTop: $(target).offset().top - 50
      }, 1000);
  });

});

