document.addEventListener('DOMContentLoaded', function() {
    const sections = document.querySelectorAll('.section');
    let currentSection = 0;

    function scrollToSection(index) {
        sections[index].scrollIntoView({ behavior: 'smooth' });
    }

    document.addEventListener('wheel', function(event) {
        if (event.deltaY > 0) {
            // Scroll down
            if (currentSection < sections.length - 1) {
                currentSection++;
                scrollToSection(currentSection);
            }
        } else {
            // Scroll up
            if (currentSection > 0) {
                currentSection--;
                scrollToSection(currentSection);
            }
        }
    });

    // Optional: handle keyboard arrow keys
    document.addEventListener('keydown', function(event) {
        if (event.key === 'ArrowDown') {
            if (currentSection < sections.length - 1) {
                currentSection++;
                scrollToSection(currentSection);
            }
        } else if (event.key === 'ArrowUp') {
            if (currentSection > 0) {
                currentSection--;
                scrollToSection(currentSection);
            }
        }
    });
});
