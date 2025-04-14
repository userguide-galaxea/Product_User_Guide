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
                        <img src="assets/GALAXEA_A1XY_title.png" alt="A1XY_title">
                        <a href="Guide/A1XY/A1XY_Getting_Started" class="btn btn-primary">了解更多</a>
                    </div>
                    <div class="image-container">
                        <img src="assets/A1XY_banner.png" alt="R1_product" class="responsive-image">
                    </div>
                </div>
            </div>
            <div class="swiper-slide">
                <div class="container">
                    <div class="text-button-container">
                        <img src="assets/R1_title.png" alt="R1_title">
                        <a href="Introducing_Galaxea_Robot/product_info/R1" class="btn btn-primary">了解更多</a>
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
                        <a href="Introducing_Galaxea_Robot/product_info/A1" class="btn btn-primary">了解更多</a>
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
                <h2><img src="assets/R1Pro_title.png" alt="R1 Pro" width="130"></h2>
                <div class="product">
                        <a href="Guide/R1Pro/R1Pro_Unbox_Startup_Guide">开箱启动</a> <br>
                        <a href="Guide/R1Pro/R1Pro_Hardware_Introduction">硬件介绍</a> <br>
                        <a href="Guide/R1Pro/R1Pro_Software_Introduction">软件介绍</a> <br>
                </div>
            </section>
            <section class="products-section">
                <h2><img src="assets/R1Lite_title.png" alt="R1 Lite" width="120"></h2>
                <div class="product">
                        <a href="Guide/R1Lite/R1Lite_Unbox_Startup_Guide">开箱启动</a> <br>
                        <a href="Guide/R1Lite/R1Lite_Hardware_Introduction">硬件介绍</a> <br>
                        <a href="Guide/R1Lite/R1Lite_Software_Introduction">软件介绍</a> <br>
                </div>
            </section>
            <section class="products-section">
                <h2><img src="assets/R1_series.png" alt="R1" width="50"></h2>
                <div class="product">
                        <a href="Guide/R1/R1_Step_by_Step_Guide">开箱启动</a> <br>
                        <a href="Guide/R1/R1_Hardware_Guide">硬件介绍</a> <br>
                        <a href="Guide/R1/R1_Software_Guide">软件介绍</a> <br>
                </div>
            </section>
            <section class="products-section">
                <h2><img src="assets/A1XY_title.png" alt="A1XY" width="120"></h2>
                <div class="product">
                        <a href="Guide/A1XY/A1XY_Unbox_Guide">开箱启动</a> <br>
                        <a href="Guide/A1XY/A1XY_Hardware_Introduction">硬件介绍</a> <br>
                        <a href="Guide/A1XY/A1XY_Software_Introduction">软件介绍</a> <br>
                </div>
            </section>
            <section class="products-section">
                <h2><img src="assets/A1_series.png" alt="A1" width="50"></h2>
                <div class="product">
                        <a href="Guide/A1/A1_Getting_Started">开箱启动</a> <br>
                        <a href="Guide/A1/A1_Hardware_Guide">硬件介绍</a> <br>
                        <a href="Guide/A1/A1_Software_Guide">软件介绍</a> <br>
                </div>
            </section>
        </div>
    </main>
    <section class="contact-section">
        <h2>联系我们</h2>
        <p>联系电话：4008 780 980 </br>邮箱（产品/商务）: <a href="mailto:product@galaxea.ai">product@galaxea.ai</a></br>邮箱（技术支持）: <a href="mailto:support@galaxea.ai">support@galaxea.ai</a></p>
    </section>
</html>

