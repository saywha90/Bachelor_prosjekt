# USN BSc LaTeX Template

This repository contains a **LaTeX report template** intended for Bachelor’s thesis and project reports at the **University of South-Eastern Norway (USN)**.

The template provides a **suggested project structure**, not a formal requirement. Its purpose is to help students write well-structured, maintainable documentation, especially for larger reports and group projects.

The structure is based on the LaTeX project used for the *Sortify* BSc project documentation (2023): https://hdl.handle.net/11250/3099988

Names and person photos in this template have been anonymized or replaced.

**Author:** Ruben Sørensen

---

## Who is this template for?

This template is intended for:
- BSc students with little to moderate LaTeX experience
- Individual or group projects
- Reports that are expected to grow large and require restructuring over time

The focus is on **clarity, scalability, and separation of content and style**.

---

## Project structure overview

The project is structured as a **hierarchy of sections and subsections**, where each level is responsible only for importing its *direct children*.

Import statements are therefore **recursive and local**, rather than centralized in one file.

### Entry point (`main.tex`)

The root `main.tex` file defines:
- the document class
- global packages and styling
- front matter (cover page, title page, abstract, etc.)
- bibliography and appendices

For the main body of the report, it imports **only one file**:

```latex
\import{./src/sections/}{main}
```

The root file does **not** directly import individual sections or subsections.

---

### Sections (`src/sections/`)

The file `src/sections/main.tex` is responsible for:
- defining the *order of sections*
- importing each section’s own `main.tex`

Example:

```latex
\subimport{./introduction/}{main}
\subimport{./some_section/}{main}
\subimport{./another_section/}{main}
```

Reordering sections is therefore done **by moving import statements in this file**, without touching section contents.

---

### Individual sections

Each section lives in its own directory and contains a `main.tex` file.  
This file:
- defines the section heading
- imports the subsections belonging to that section

Example:

```latex
\hmsection{Some section}{OH}{IO}

\subimport{./}{some_subsection}
\subimport{./}{some_other_subsection}
```

Subsections are fully contained within the section directory.

---

### Subsections and content files

Subsections are regular `.tex` files that contain only content.  
They may freely include:
- figures
- references
- glossary entries
- local resources

Because the `import` / `subimport` mechanism is used, resources can be referenced **relative to the local directory**, for example:

```latex
\includegraphics{res/xp_wallpaper.jpg}
```

---

### Resources

Each section may contain its own `res/` directory for images, PDFs, and other assets.  
Keeping resources close to where they are used:

- avoids naming conflicts
- improves portability
- keeps directories manageable

---

### Styling

All document styling is contained in `ReportStyle.sty`.

No formatting logic is scattered throughout the content files, which ensures:
- consistent formatting
- easier maintenance
- clear separation between *content* and *presentation*

---

## Compiling the document

### Overleaf

Overleaf is an online LaTeX editor and is recommended for:
- beginners
- group work
- collaborative writing

This template assumes **`xelatex`** is used as the compiler (required for font handling and Unicode support).

In Overleaf project settings:
1. Set the **compiler** to `XeLaTeX`
2. Set `main.tex` as the **Main document**

A preconfigured Overleaf project (read-only) is available here:  
https://www.overleaf.com/read/dfrbtbwntzcc#f3faf6

You may use it as a reference, duplicate it, or inspect the settings.  
Note that the availability of this link cannot be guaranteed indefinitely.

#### Note on compilation warnings and errors

Overleaf is intentionally very forgiving when compiling LaTeX documents.  
Even with multiple errors or warnings, it will often still produce a PDF.

While this can be convenient, it may also hide underlying problems in the document.
Compilation errors and warnings can lead to:
- inconsistent formatting
- missing or misplaced figures
- incorrect references
- subtle errors in the final content

**Recommendation:**  
Aim for a build with **zero errors and zero warnings**.

---

### Local compilation (Linux)

If you prefer working locally, a `Makefile` is included.

#### Requirements

You must have a LaTeX distribution installed. Examples:

- **Fedora**
  ```bash
  sudo dnf install texlive-scheme-full
  ```
- **Ubuntu**
  ```bash
  sudo apt-get install texlive texstudio
  ```

**NOTE:** These packages install the full LaTeX suite and may require significant disk space.

#### Compiling

To compile the report locally, run:

```bash
make
```

To remove auxiliary build files:

```bash
make clean
```

---

## Notes

- This template focuses on **structure**, not content
- You are free to remove, rename, or reorganize sections as needed
- The structure scales well for large reports and appendices

Feel free to adapt the template to your project’s needs.
