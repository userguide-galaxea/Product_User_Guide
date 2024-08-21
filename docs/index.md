---
hide:
  - navigation
  - toc
---

#    

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Swiper Example</title>
    <!-- Link Swiper's CSS -->
    <link rel="stylesheet" href="https://unpkg.com/swiper/swiper-bundle.min.css">
    <!-- Link to custom CSS file -->
    <link rel="stylesheet" href="styles.css">
</head>
<body>
    <div class="swiper-container">
        <div class="swiper-wrapper">
            <div class="swiper-slide">
                <div class="container">
                    <div class="text-button-container">
                        <img src="assets/R1_title.png" alt="R1_title">
                        <a href="Introducing_Galaxea_Robot/product_info/R1" class="btn btn-primary">Learn More</a>
                    </div>
                    <div class="image-container">
                        <img src="assets/R1_product.png" alt="R1_product" class="responsive-image">
                    </div>
                </div>
            </div>
            <div class="swiper-slide">
                <div class="container">
                    <div class="text-button-container">
                        <img src="assets/A1_title.png" alt="A1_title">
                        <a href="Introducing_Galaxea_Robot/product_info/A1" class="btn btn-primary">Learn More</a>
                    </div>
                    <div class="image-container">
                        <img src="assets/temp.png" alt="A1_product">
                    </div>
                </div>
            </div>
        </div>
        <div class="swiper-pagination"></div>
        <div class="swiper-button-next"></div>
        <div class="swiper-button-prev"></div>
    </div>
    <script src="https://unpkg.com/swiper/swiper-bundle.min.js"></script>
    <script>
        var swiper = new Swiper('.swiper-container', {
            direction: 'horizontal',
            effect: 'fade',
            loop: true, 
            pagination: {
                el: '.swiper-pagination',
                clickable: true,
            },
            navigation: {
                nextEl: '.swiper-button-next',
                prevEl: '.swiper-button-prev',
            },
            fadeEffect: {
                crossFade: true
            }
        });
    </script>
</body>
    <main id = unique-page>
        <div class="row">
            <section class="products-section">
                <h2><img src="assets/R1_series.png" alt="R1" width="50"></h2>
                <div class="product">
                        <a href="Guide/R1/GettingStarted_Before">Getting Started</a> <br>
                        <a href="Guide/R1/Hardware_Guide">Hardware Guide</a> <br>
                        <a href="Guide/R1/Software_Guide">Software Guide</a> <br>
                </div>
            </section>
            <section class="products-section">
                <h2><img src="assets/A1_series.png" alt="A1" width="50"></h2>
                <div class="product">
                        <a href="Guide/A1/Getting_Started">Getting Started</a> <br>
                        <a href="Guide/A1/Hardware_Guide">Hardware Guide</a> <br>
                        <a href="Guide/A1/Software_Guide">Software Guide</a> <br>
                </div>
            </section>
        </div>
    </main>
    <section class="contact-section">
        <h2>Contact Us</h2>
        <p>Email: <a href="mailto:info@galaxea.ai">info@galaxea.ai</a></p>
    </section>
</html>
